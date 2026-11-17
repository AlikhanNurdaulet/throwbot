"""Hardware access for the SO-101 follower (and leader), plus a simulated arm for --dry-run.

All joint values are in degrees (LeRobot `use_degrees=True`), except the gripper, which is 0..100
(0 = closed, 100 = fully open, as set by your LeRobot calibration).
"""

from __future__ import annotations

import time

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ARM_JOINTS = JOINTS[:-1]


def _import_follower():
    """Find the SO-101 follower class across LeRobot versions."""
    attempts = [
        ("lerobot.robots.so_follower", "SO101Follower", "SO101FollowerConfig"),  # lerobot >= 0.5
        ("lerobot.robots.so101_follower", "SO101Follower", "SO101FollowerConfig"),  # 0.2 - 0.4
        ("lerobot.common.robots.so101_follower", "SO101Follower", "SO101FollowerConfig"),  # older
    ]
    for mod, cls, cfg in attempts:
        try:
            m = __import__(mod, fromlist=[cls, cfg])
            return getattr(m, cls), getattr(m, cfg)
        except (ImportError, AttributeError):
            continue
    raise ImportError("Could not import SO101Follower from lerobot. Is lerobot installed with the feetech extra?")


def _import_leader():
    attempts = [
        ("lerobot.teleoperators.so_leader", "SO101Leader", "SO101LeaderConfig"),
        ("lerobot.teleoperators.so101_leader", "SO101Leader", "SO101LeaderConfig"),
        ("lerobot.common.teleoperators.so101_leader", "SO101Leader", "SO101LeaderConfig"),
    ]
    for mod, cls, cfg in attempts:
        try:
            m = __import__(mod, fromlist=[cls, cfg])
            return getattr(m, cls), getattr(m, cfg)
        except (ImportError, AttributeError):
            continue
    raise ImportError("Could not import SO101Leader from lerobot.")


def _make_config(cfg_cls, **kwargs):
    """Build a LeRobot config, dropping fields an older LeRobot version doesn't know."""
    try:
        return cfg_cls(**kwargs)
    except TypeError:
        kwargs.pop("use_degrees", None)
        kwargs.pop("max_relative_target", None)
        return cfg_cls(**kwargs)


def _strip(d: dict) -> dict:
    return {k.removesuffix(".pos"): float(v) for k, v in d.items() if k.endswith(".pos")}


class RealArm:
    """Thin wrapper around LeRobot's SO101Follower."""

    dry_run = False

    def __init__(self, port: str, robot_id: str):
        Follower, FollowerConfig = _import_follower()
        # max_relative_target must be None: it would clip the fast throw. Our own safety checks
        # (joint limits, speed limits, confirmations) replace it.
        cfg = _make_config(FollowerConfig, port=port, id=robot_id, use_degrees=True, max_relative_target=None)
        if not getattr(cfg, "use_degrees", True):
            print("WARNING: this LeRobot version has no use_degrees option. Angles are in -100..100 units, "
                  "not degrees. Re-record all poses with `pose` before throwing.")
        self.robot = Follower(cfg)

    def connect(self):
        # calibrate=False: never start a calibration by surprise. Calibrate with lerobot-calibrate first.
        self.robot.connect(calibrate=False)
        if not self.robot.is_calibrated:
            raise RuntimeError("Follower is not calibrated (or the id doesn't match). Run lerobot-calibrate first.")

    def read(self) -> dict[str, float]:
        return {k: float(v) for k, v in self.robot.bus.sync_read("Present_Position").items()}

    def write(self, goal: dict[str, float]):
        self.robot.bus.sync_write("Goal_Position", goal)

    def temperatures(self) -> dict[str, float]:
        bus = self.robot.bus
        try:
            try:
                return {k: float(v) for k, v in bus.sync_read("Present_Temperature", normalize=False).items()}
            except TypeError:
                return {k: float(v) for k, v in bus.sync_read("Present_Temperature").items()}
        except Exception as e:  # noqa: BLE001 - temperature is nice to have, never fatal
            print(f"(could not read temperatures: {e})")
            return {}

    def torque(self, on: bool):
        if on:
            self.robot.bus.enable_torque()
        else:
            self.robot.bus.disable_torque()

    def disconnect(self, torque_off: bool = True):
        try:
            self.robot.bus.disconnect(torque_off)
        except TypeError:
            self.robot.disconnect()


class DryRunArm:
    """Simulated arm. Each joint chases its goal at a capped speed, roughly like an STS3215 at 12 V."""

    dry_run = True

    def __init__(self, start: dict[str, float], max_speed_deg_s: float = 270.0):
        self.pos = dict(start)
        self.goal = dict(start)
        self.max_speed = max_speed_deg_s
        self.t = time.perf_counter()

    def connect(self):
        print("[dry-run] simulated arm connected (no hardware is touched)")

    def _step(self):
        now = time.perf_counter()
        dt, self.t = now - self.t, now
        for j, g in self.goal.items():
            speed = 600.0 if j == "gripper" else self.max_speed  # gripper in % per second
            d = g - self.pos[j]
            step = max(-speed * dt, min(speed * dt, d))
            self.pos[j] += step

    def read(self):
        self._step()
        return dict(self.pos)

    def write(self, goal):
        self._step()
        self.goal.update(goal)

    def temperatures(self):
        return {j: 30.0 for j in JOINTS}

    def torque(self, on):
        print(f"[dry-run] torque {'on' if on else 'off'}")

    def disconnect(self, torque_off=True):
        print("[dry-run] disconnected")


class Leader:
    def __init__(self, port: str, leader_id: str):
        LeaderCls, LeaderConfig = _import_leader()
        self.dev = LeaderCls(_make_config(LeaderConfig, port=port, id=leader_id, use_degrees=True))

    def connect(self):
        self.dev.connect(calibrate=False)

    def read(self):
        return _strip(self.dev.get_action())

    def disconnect(self):
        self.dev.disconnect()
