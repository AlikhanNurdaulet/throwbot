#!/usr/bin/env python
"""Scripted ping pong ball throw for the SO-101 follower arm.

Commands (add --dry-run to any of them to simulate without hardware):
  python throwarm.py check                       read joints + temperatures, validate config
  python throwarm.py throw front                 one throw at a cup
  python throwarm.py tune front                  throw, log the result, adjust, repeat
  python throwarm.py repeat front back_left -n 10   N logged throws per cup + hit rate
  python throwarm.py pose --source follower      record poses by moving the arm by hand
  python throwarm.py pose --source leader        record poses with the leader arm (follower mirrors it)
  python throwarm.py rest                        move slowly to the rest pose
  python throwarm.py stats                       hit rate per cup and per setting from the log
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

from arm import ARM_JOINTS, JOINTS, DryRunArm, Leader, RealArm

HERE = Path(__file__).resolve().parent

RESULT_HELP = ("Result: [h]it, [b]ounced out/rim, [s]hort, [l]ong, [<] left, [>] right "
               "(combine, e.g. 's<' = short and left), [x] bad throw/ignore, Enter = skip")


# ----------------------------------------------------------------------------- config

def deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def parse_sets(sets: list[str] | None) -> dict:
    """--set a=1 --set windup.elbow_flex=70  ->  {'a': 1, 'windup': {'elbow_flex': 70}}"""
    out: dict = {}
    for s in sets or []:
        if "=" not in s:
            raise SystemExit(f"--set needs key=value, got {s!r}")
        key, val = s.split("=", 1)
        node = out
        parts = key.strip().split(".")
        for p in parts[:-1]:
            node = node.setdefault(p, {})
        node[parts[-1]] = yaml.safe_load(val)
    return out


def load_config(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def cup_params(cfg: dict, cup: str, overrides: dict | None = None) -> dict:
    if cup not in cfg["cups"]:
        raise SystemExit(f"Unknown cup {cup!r}. Cups in config: {', '.join(cfg['cups'])}")
    p = deep_merge(cfg["throw_defaults"], cfg["cups"][cup] or {})
    p = deep_merge(p, overrides or {})
    p.setdefault("pan", 0)
    return p


def build_poses(p: dict) -> tuple[dict, dict]:
    """Full 6-joint wind-up and follow-through poses for a cup's params."""
    closed = p["gripper_closed"] if p["release_mode"] == "gripper" else None
    windup = {"shoulder_pan": p["pan"], **p["windup"]}
    follow = {"shoulder_pan": p["pan"], **p["follow_through"]}
    for pose in (windup, follow):
        missing = [j for j in ARM_JOINTS if j not in pose]
        if missing:
            raise SystemExit(f"Pose is missing joints {missing}. Fill in windup/follow_through in config.yaml.")
        if closed is not None:
            pose["gripper"] = closed
    return windup, follow


# ----------------------------------------------------------------------------- safety

def check_pose(cfg: dict, pose: dict, name: str):
    lim = cfg["safety"]["joint_limits"]
    bad = [f"{j}={v:.1f} (limit {lim[j]})" for j, v in pose.items()
           if j in lim and not (lim[j][0] <= v <= lim[j][1])]
    if bad:
        raise SystemExit(f"REFUSED: {name} pose is outside safety.joint_limits: {', '.join(bad)}")


def check_throw(cfg: dict, p: dict, windup: dict, follow: dict):
    s = cfg["safety"]
    check_pose(cfg, windup, "wind-up")
    check_pose(cfg, follow, "follow-through")
    T = float(p["throw_time_s"])
    if T < s["min_throw_time_s"]:
        raise SystemExit(f"REFUSED: throw_time_s={T} is below safety.min_throw_time_s={s['min_throw_time_s']}")
    speeds = {j: abs(follow[j] - windup[j]) / T for j in ARM_JOINTS}
    fastest = max(speeds, key=speeds.get)
    # ease_in peaks at 2x the average speed
    peak = speeds[fastest] * (2.0 if p.get("profile") == "ease_in" else 1.0)
    if peak > s["max_throw_speed_deg_s"]:
        raise SystemExit(f"REFUSED: {fastest} would be commanded at {peak:.0f} deg/s, above "
                         f"safety.max_throw_speed_deg_s={s['max_throw_speed_deg_s']}")
    rj, ra = p["release_joint"], float(p["release_angle"])
    lo, hi = sorted((windup[rj], follow[rj]))
    if p["release_mode"] == "gripper" and not (lo <= ra <= hi):
        raise SystemExit(f"REFUSED: release_angle={ra} is not between the wind-up ({windup[rj]}) and "
                         f"follow-through ({follow[rj]}) values of {rj}, so it would never trigger.")
    if p["release_mode"] == "gripper" and abs(follow[rj] - windup[rj]) < 1:
        raise SystemExit(f"REFUSED: {rj} barely moves during the throw, so it can't trigger the release.")


def check_temps(cfg: dict, arm) -> float:
    temps = arm.temperatures()
    if not temps:
        return float("nan")
    hottest = max(temps, key=temps.get)
    t = temps[hottest]
    if t >= cfg["safety"]["max_temperature_c"]:
        raise SystemExit(f"REFUSED: {hottest} is at {t:.0f} C (limit {cfg['safety']['max_temperature_c']} C). "
                         "Let the servos cool down.")
    return t


# ----------------------------------------------------------------------------- motion

def move_slow(cfg: dict, arm, target: dict, speed: float | None = None):
    """Smooth, speed-limited move (cosine profile) to a target pose."""
    check_pose(cfg, target, "target")
    speed = speed or cfg["safety"]["slow_speed_deg_s"]
    start = arm.read()
    dist = max([abs(target[j] - start[j]) for j in target if j != "gripper"] or [0.0])
    dur = max(0.3, dist / speed)
    dt = 1.0 / cfg["control"]["rate_hz"]
    t0 = time.perf_counter()
    while True:
        s = min(1.0, (time.perf_counter() - t0) / dur)
        k = 0.5 - 0.5 * math.cos(math.pi * s)
        arm.write({j: start[j] + (target[j] - start[j]) * k for j in target})
        if s >= 1.0:
            break
        time.sleep(dt)
    time.sleep(0.2)


def hold_here(arm):
    """Freeze the arm where it is (used on Ctrl-C)."""
    try:
        arm.write(arm.read())
    except Exception as e:  # noqa: BLE001
        print(f"(could not hold position: {e})")


def do_throw(cfg: dict, arm, p: dict, windup: dict, follow: dict) -> dict:
    """The fast part. Assumes the arm is already at the wind-up pose with the ball loaded.

    Streams interpolated targets wind-up -> follow-through at control.rate_hz and reads the real
    joint angles every tick. The gripper opens when the *measured* release_joint angle (predicted
    release_lead_s ahead) crosses release_angle, so the release point stays the same even when the
    servos lag behind the commanded trajectory.
    """
    T = float(p["throw_time_s"])
    rj, ra = p["release_joint"], float(p["release_angle"])
    direction = 1.0 if follow[rj] >= windup[rj] else -1.0
    lead = float(p.get("release_lead_s", 0.0))
    use_gripper = p["release_mode"] == "gripper"
    ease_in = p.get("profile") == "ease_in"
    dt = 1.0 / cfg["control"]["rate_hz"]

    info = {"release_angle_actual": None, "release_vel_deg_s": None, "release_t_s": None,
            "peak_vel_deg_s": 0.0, "loop_hz": None}
    released = not use_gripper
    grip = p["gripper_closed"] if use_gripper else None
    prev_a, prev_t = arm.read()[rj], 0.0
    vel = 0.0
    ticks = 0
    t0 = time.perf_counter()
    while True:
        t = time.perf_counter() - t0
        s = min(1.0, t / T)
        k = s * s if ease_in else s
        goal = {j: windup[j] + (follow[j] - windup[j]) * k for j in ARM_JOINTS}
        if grip is not None:
            goal["gripper"] = grip
        arm.write(goal)

        a = arm.read()[rj]
        if t > prev_t:
            inst = (a - prev_a) / (t - prev_t)
            vel = 0.5 * vel + 0.5 * inst  # light smoothing of the noisy derivative
        prev_a, prev_t = a, t
        info["peak_vel_deg_s"] = max(info["peak_vel_deg_s"], abs(vel))
        ticks += 1

        if not released:
            predicted = a + vel * lead
            if (predicted - ra) * direction >= 0:
                grip = p["gripper_open"]
                arm.write({"gripper": grip})
                released = True
                info.update(release_angle_actual=round(a, 2), release_vel_deg_s=round(vel, 1),
                            release_t_s=round(t, 3))
            elif s >= 1.0 and t > T + 0.5:
                # The arm stalled before the release angle. Let go anyway so it doesn't hold the ball forever.
                print(f"WARNING: {rj} never reached release_angle={ra} (stuck at {a:.1f}). Releasing now.")
                grip = p["gripper_open"]
                arm.write({"gripper": grip})
                released = True

        if s >= 1.0 and released and t > T + float(p.get("settle_s", 0.5)):
            break
        time.sleep(max(0.0, dt - (time.perf_counter() - t0 - t)))

    info["loop_hz"] = round(ticks / (time.perf_counter() - t0), 1)
    info["peak_vel_deg_s"] = round(info["peak_vel_deg_s"], 1)
    return info


def ask(prompt: str, auto: str | None = None) -> str:
    if auto is not None:
        print(f"{prompt}{auto}")
        return auto
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def throw_once(cfg: dict, arm, cup: str, p: dict, yes: bool) -> dict:
    windup, follow = build_poses(p)
    check_throw(cfg, p, windup, follow)
    temp = check_temps(cfg, arm)
    auto = "" if arm.dry_run else None

    # Go to wind-up with the gripper open, load the ball, close.
    if p["release_mode"] == "gripper":
        move_slow(cfg, arm, {**windup, "gripper": p["gripper_open"]})
        ask("Place the ball in the gripper, then press Enter to close it... ", auto)
        move_slow(cfg, arm, windup)
    else:
        move_slow(cfg, arm, windup)
        ask("Load the ball, then press Enter... ", auto)

    if cfg["safety"]["confirm_each_throw"] and not yes:
        r = ask(f"[{cup}] Area clear? Enter = THROW, n = cancel: ", auto)
        if r.lower().startswith("n"):
            print("Cancelled.")
            move_slow(cfg, arm, cfg["rest"])
            return {"cancelled": True}

    print(f"Throwing at {cup}: throw_time_s={p['throw_time_s']} release {p['release_joint']}"
          f"@{p['release_angle']} pan={p['pan']}")
    info = do_throw(cfg, arm, p, windup, follow)
    print(f"  released at {info['release_angle_actual']} deg, {info['release_vel_deg_s']} deg/s "
          f"(t={info['release_t_s']} s), peak {info['peak_vel_deg_s']} deg/s, loop {info['loop_hz']} Hz")
    move_slow(cfg, arm, {**cfg["rest"], "shoulder_pan": p["pan"]})
    info["max_temp_c"] = temp
    return info


# ----------------------------------------------------------------------------- logging

LOG_FIELDS = ["timestamp", "mode", "cup", "trial", "result", "hit", "throw_time_s", "release_joint",
              "release_angle", "release_angle_actual", "release_vel_deg_s", "release_t_s",
              "peak_vel_deg_s", "loop_hz", "max_temp_c", "pan", "dry_run", "note", "params_json"]


def parse_result(r: str) -> str | None:
    r = r.strip()
    if not r:
        return None
    if r.lower() in ("h", "hit"):
        return "hit"
    if r.lower() in ("b", "rim"):
        return "rim"
    if r.lower() in ("x",):
        return "ignore"
    parts = []
    if "s" in r.lower():
        parts.append("short")
    if "l" in r.lower():
        parts.append("long")
    if "<" in r:
        parts.append("left")
    if ">" in r:
        parts.append("right")
    return "-".join(parts) or None


def log_throw(cfg: dict, mode: str, cup: str, trial: int, p: dict, info: dict, result: str | None,
              note: str, dry_run: bool):
    path = HERE / cfg.get("log_file", "throws.csv")
    new = not path.exists()
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=LOG_FIELDS)
        if new:
            w.writeheader()
        w.writerow({
            "timestamp": datetime.now().isoformat(timespec="seconds"), "mode": mode, "cup": cup,
            "trial": trial, "result": result or "", "hit": int(result == "hit") if result and result not in ("ignore", "dry-run") else "",
            "throw_time_s": p["throw_time_s"], "release_joint": p["release_joint"],
            "release_angle": p["release_angle"], "pan": p["pan"], "dry_run": int(dry_run), "note": note,
            "params_json": json.dumps(p, sort_keys=True),
            **{k: info.get(k) for k in ("release_angle_actual", "release_vel_deg_s", "release_t_s",
                                        "peak_vel_deg_s", "loop_hz", "max_temp_c")},
        })


def ask_result(dry_run: bool) -> tuple[str | None, str]:
    if dry_run:
        return "dry-run", ""
    print(RESULT_HELP)
    res = parse_result(ask("  result> "))
    note = ask("  note (optional)> ")
    return res, note


# ----------------------------------------------------------------------------- commands

def make_arm(cfg: dict, dry_run: bool):
    if dry_run:
        arm = DryRunArm(start=dict(cfg["rest"]))
    else:
        arm = RealArm(cfg["robot"]["port"], cfg["robot"]["id"])
    arm.connect()
    return arm


def cmd_check(cfg, args):
    arm = make_arm(cfg, args.dry_run)
    try:
        pos = arm.read()
        print("Joints:", {j: round(v, 1) for j, v in pos.items()})
        print("Temps C:", arm.temperatures())
        check_pose(cfg, pos, "current")
        for cup in cfg["cups"]:
            p = cup_params(cfg, cup)
            w, f = build_poses(p)
            check_throw(cfg, p, w, f)
            print(f"  {cup}: config OK")
    finally:
        arm.disconnect(torque_off=True)


def cmd_rest(cfg, args):
    arm = make_arm(cfg, args.dry_run)
    try:
        move_slow(cfg, arm, cfg["rest"])
    finally:
        arm.disconnect(torque_off=True)


def run_session(cfg, args, mode: str):
    arm = make_arm(cfg, args.dry_run)
    overrides = parse_sets(args.set)
    try:
        move_slow(cfg, arm, cfg["rest"])
        if mode == "throw":
            p = cup_params(cfg, args.cups[0], overrides)
            info = throw_once(cfg, arm, args.cups[0], p, args.yes)
            if args.log and not info.get("cancelled"):
                res, note = ask_result(args.dry_run)
                log_throw(cfg, "throw", args.cups[0], 1, p, info, res, note, args.dry_run)

        elif mode == "tune":
            cup = args.cups[0]
            trial = 0
            while True:
                p = cup_params(cfg, cup, overrides)
                trial += 1
                info = throw_once(cfg, arm, cup, p, args.yes)
                if not info.get("cancelled"):
                    res, note = ask_result(args.dry_run)
                    log_throw(cfg, "tune", cup, trial, p, info, res, note, args.dry_run)
                if args.dry_run and trial >= 2:
                    break
                print("Enter = same again | key=value (e.g. throw_time_s=0.26 or windup.elbow_flex=65) "
                      "to change | show | q = quit")
                while True:
                    cmd = ask("tune> ")
                    if cmd in ("q", "quit", "exit"):
                        return
                    if cmd == "show":
                        print(yaml.safe_dump(cup_params(cfg, cup, overrides), sort_keys=False))
                        continue
                    if cmd:
                        try:
                            overrides = deep_merge(overrides, parse_sets(cmd.split()))
                            p2 = cup_params(cfg, cup, overrides)
                            check_throw(cfg, p2, *build_poses(p2))
                            print("  now:", json.dumps(overrides))
                        except SystemExit as e:
                            print(e)
                            continue
                    break
                if overrides:
                    print(f"  (when happy, copy these into cups.{cup} in config.yaml: {json.dumps(overrides)})")

        elif mode == "repeat":
            results: dict[str, list[int]] = {}
            for cup in args.cups:
                p = cup_params(cfg, cup, overrides)
                for i in range(1, args.n + 1):
                    print(f"\n=== {cup} throw {i}/{args.n} ===")
                    info = throw_once(cfg, arm, cup, p, args.yes)
                    if info.get("cancelled"):
                        continue
                    res, note = ask_result(args.dry_run)
                    log_throw(cfg, "repeat", cup, i, p, info, res, note, args.dry_run)
                    if res and res not in ("ignore", "dry-run"):
                        results.setdefault(cup, []).append(int(res == "hit"))
                    if args.cooldown > 0 and not args.dry_run:
                        time.sleep(args.cooldown)
            print("\nHit rate:")
            for cup, hits in results.items():
                print(f"  {cup}: {sum(hits)}/{len(hits)} ({100 * sum(hits) / len(hits):.0f}%)")

        move_slow(cfg, arm, cfg["rest"])
    except KeyboardInterrupt:
        print("\nStopped. Holding position.")
        hold_here(arm)
        if ask("Press Enter to move slowly to rest (or type 'stay' to leave torque on and exit): ") != "stay":
            move_slow(cfg, arm, cfg["rest"])
            arm.disconnect(torque_off=True)
        else:
            arm.disconnect(torque_off=False)
        return
    arm.disconnect(torque_off=True)


def cmd_pose(cfg, args):
    """Record poses. follower: torque off, move it by hand. leader: follower mirrors the leader."""
    out = HERE / "recorded_poses.yaml"
    arm = make_arm(cfg, args.dry_run)
    leader = None
    try:
        if args.source == "leader":
            if args.dry_run:
                raise SystemExit("Leader recording needs hardware.")
            leader = Leader(cfg["leader"]["port"], cfg["leader"]["id"])
            leader.connect()
            print("Move the LEADER arm. The follower mirrors it slowly. Ctrl-C to stop mirroring and quit.")
            print("Start with the leader near the follower's current pose!")
            first = leader.read()
            move_slow(cfg, arm, first)  # catch up slowly before fast mirroring
        else:
            print("Torque OFF: hold the follower arm, it will go limp.")
            ask("Press Enter when you're holding it... ", "" if args.dry_run else None)
            arm.torque(False)

        print("Type a name and press Enter to save the current pose (e.g. windup, follow_through, rest). "
              "Enter alone just prints it. q = quit.")
        import threading
        stop = threading.Event()
        latest: dict = {}

        def mirror():
            # Only this thread talks to the leader bus. Follower moves are capped at 120 deg/s.
            dt, max_step = 1.0 / 50, 120.0 / 50
            cmd = arm.read()
            while not stop.is_set():
                target = leader.read()
                latest.update(target)
                try:
                    check_pose(cfg, target, "leader")
                    cmd = {j: cmd[j] + max(-max_step, min(max_step, target[j] - cmd[j])) for j in target}
                    arm.write(cmd)
                except SystemExit as e:
                    print(f"\r{e} (not sent)", end="")
                time.sleep(dt)

        th = None
        if leader:
            th = threading.Thread(target=mirror, daemon=True)
            th.start()
        while True:
            name = ask("pose name> ", "q" if args.dry_run else None)
            if name == "q":
                break
            pose = dict(latest) if leader else arm.read()
            pose = {j: round(pose[j], 1) for j in JOINTS if j in pose}
            print(yaml.safe_dump({name or "pose": pose}, sort_keys=False))
            if name:
                existing = yaml.safe_load(out.read_text()) if out.exists() else {}
                existing = existing or {}
                existing[name] = pose
                out.write_text(yaml.safe_dump(existing, sort_keys=False))
                print(f"  saved to {out.name}. Copy it into config.yaml (drop shoulder_pan/gripper for "
                      "windup/follow_through, use shoulder_pan as the cup's pan).")
        stop.set()
        if th:
            th.join(timeout=1)
    except KeyboardInterrupt:
        pass
    finally:
        if leader:
            leader.disconnect()
            hold_here(arm)
        else:
            print("Hold the arm: torque is coming back on at the current pose.")
            ask("Press Enter when ready... ", "" if args.dry_run else None)
            hold_here(arm)
            arm.torque(True)
        move_slow(cfg, arm, cfg["rest"])
        arm.disconnect(torque_off=True)


def cmd_stats(cfg, args):
    path = HERE / cfg.get("log_file", "throws.csv")
    if not path.exists():
        raise SystemExit(f"No log yet at {path}")
    rows = [r for r in csv.DictReader(open(path)) if r["hit"] != "" and r["dry_run"] == "0"]
    if not rows:
        print("No logged real throws with a result yet.")
        return
    by_cup: dict = {}
    for r in rows:
        key = (r["cup"], r["throw_time_s"], r["release_angle"], r["pan"])
        by_cup.setdefault(key, []).append(r)
    print(f"{'cup':<12}{'throw_time':>11}{'release':>9}{'pan':>6}{'hits':>8}{'rate':>7}  misses")
    for (cup, tt, ra, pan), rs in sorted(by_cup.items()):
        hits = sum(int(r["hit"]) for r in rs)
        misses: dict = {}
        for r in rs:
            if r["result"] != "hit":
                misses[r["result"]] = misses.get(r["result"], 0) + 1
        miss = ", ".join(f"{k} x{v}" for k, v in sorted(misses.items(), key=lambda x: -x[1]))
        print(f"{cup:<12}{tt:>11}{ra:>9}{pan:>6}{hits:>5}/{len(rs):<3}{100 * hits / len(rs):>5.0f}%  {miss}")
    print("\n(Only rows where every other setting was the same are truly comparable; see params_json.)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(HERE / "config.yaml"))
    ap.add_argument("--dry-run", action="store_true", help="simulate; never touches the hardware")
    ap.add_argument("--yes", action="store_true", help="skip the 'area clear?' confirmation")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("check")
    sub.add_parser("rest")
    sub.add_parser("stats")

    for name in ("throw", "tune"):
        sp = sub.add_parser(name)
        sp.add_argument("cups", nargs=1)
        sp.add_argument("--set", action="append", help="override a setting, e.g. throw_time_s=0.22")
        if name == "throw":
            sp.add_argument("--log", action="store_true", help="ask for the result and log it")

    sp = sub.add_parser("repeat")
    sp.add_argument("cups", nargs="+")
    sp.add_argument("-n", type=int, default=10)
    sp.add_argument("--set", action="append")
    sp.add_argument("--cooldown", type=float, default=0.0, help="seconds to wait between throws")

    sp = sub.add_parser("pose")
    sp.add_argument("--source", choices=["follower", "leader"], default="follower")

    args = ap.parse_args()
    cfg = load_config(Path(args.config))
    if args.dry_run:
        print("*** DRY RUN: simulated arm, nothing is sent to the hardware ***")

    if args.cmd == "check":
        cmd_check(cfg, args)
    elif args.cmd == "rest":
        cmd_rest(cfg, args)
    elif args.cmd == "stats":
        cmd_stats(cfg, args)
    elif args.cmd == "pose":
        cmd_pose(cfg, args)
    else:
        run_session(cfg, args, args.cmd)


if __name__ == "__main__":
    sys.exit(main())
