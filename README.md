# throwbot: scripted ping pong throw for the SO-101

One fixed throw motion, tuned per cup. The arm moves slowly to a wind-up pose, you load the ball,
then it sweeps fast to a follow-through pose and opens the gripper when a chosen joint **passes a
set angle** (not after a delay), so the release point stays the same from throw to throw.

Files:
- `throwarm.py`: the command line tool
- `arm.py`: talks to the arm through LeRobot, plus a simulated arm for `--dry-run`
- `config.yaml`: poses, cup presets and safety limits. **Everything marked `TUNE` is a placeholder.**
- `throws.csv`: created on the first logged throw
- `recorded_poses.yaml`: created by the `pose` command

## Setup

```bash
pip install 'lerobot[feetech]'      # any recent version; tested against the 0.7 API
git clone https://github.com/AlikhanNurdaulet/throwbot && cd throwbot
python throwarm.py --dry-run check  # no hardware needed: checks the config makes sense
```

In `config.yaml`, set `robot.port` (run `lerobot-find-port`) and `robot.id` to the **same id you used
for `lerobot-calibrate`**. The tool never starts a calibration on its own. Joint values are degrees
(LeRobot `use_degrees=True`); the gripper is 0 (closed) to 100 (open).

## Safety

- `--dry-run` works on every command and simulates the arm. Run each new setting there first.
- Poses outside `safety.joint_limits` are refused, not clipped. Set these limits first.
- Throws are refused if they are faster than `max_throw_speed_deg_s` or `min_throw_time_s`, if the
  release angle can't trigger, or if a servo is hotter than `max_temperature_c`.
- Every move except the throw itself runs at `slow_speed_deg_s`.
- Before every throw it asks "Area clear?" (skip with `--yes` once you trust a setting).
- Ctrl-C freezes the arm where it is, then asks before moving to rest.
- On exit the arm goes to `rest` and torque turns off. **`rest` must be a pose where the arm
  is folded and won't fall.** Record it first.
- Clamp the base down: a fast throw will move an unclamped arm.

## 1. Record the poses

```bash
python throwarm.py pose --source follower   # torque off, move the follower by hand
python throwarm.py pose --source leader     # or move the leader, the follower mirrors it (capped speed)
```

Type a name (`rest`, `windup`, `follow_through`, `aim_front`, ...) and Enter to save the current pose to
`recorded_poses.yaml`. Then copy the values into `config.yaml`:
- `rest`: all 6 joints.
- `throw_defaults.windup` / `follow_through`: shoulder_lift, elbow_flex, wrist_flex, wrist_roll
  (shoulder_pan comes from the cup, the gripper from `gripper_closed`).
- `cups.<cup>.pan`: point the arm at each cup and save the shoulder_pan value.

Pick `release_joint` (the joint that moves most during the throw, usually shoulder_lift or elbow_flex)
and set `release_angle` somewhere between its wind-up and follow-through values. Then run
`python throwarm.py check` with the arm connected.

## 2. Tune one cup at a time

```bash
python throwarm.py tune front
```

It throws, asks for the result, logs it, then waits:
- Enter: same throw again
- `throw_time_s=0.26` or `windup.elbow_flex=65`: change one setting
- `show`: print the current settings, `q`: quit

Result keys: `h` hit, `b` bounced out, `s` short, `l` long, `<` left, `>` right (combine: `s<`),
`x` bad throw (ignored in stats).

**Turn one knob at a time**, in this order:
1. **Left or right misses:** the cup's `pan`.
2. **Short or long:** `throw_time_s` (smaller = faster = farther), then `release_angle`
   (releasing later = flatter, earlier = higher lob).
3. **Release consistency:** each throw prints the angle and speed at release. If the actual
   release angle varies by more than ~2 degrees, check `release_lead_s` (the time the gripper needs to
   open; measure it from a slow motion video with the cameras) and the gripper's grip on the ball.

When a setting works, copy the overrides it prints into `cups.<cup>` in `config.yaml`.

Good to know: the STS3215 tops out around 250 to 300 deg/s at 12 V (slower on the 7.4 V version).
Below a certain `throw_time_s` the servos can't keep up, so the throw stops getting harder. The printed
`peak` speed shows when you've hit that ceiling. To go farther past that point, change the poses
(longer arc, wrist snap) rather than the time.

## 3. Check repeatability

```bash
python throwarm.py repeat front back_left back_right -n 10 --cooldown 20
python throwarm.py stats
```

`repeat` throws N times per cup with the current config and prints the hit rate. `stats` groups the log by
cup and setting and shows the most common miss. On the day, aim all 10 attempts at the cup with the best
rate (a hit is a hit). STS3215s weaken as they heat up, so test at the temperature they'll be at in
the competition (the log records the hottest servo for each throw).

## Other commands

```bash
python throwarm.py throw back_left --log   # single throw, optionally logged
python throwarm.py rest                    # slowly fold to rest
python throwarm.py --config other.yaml ... # e.g. one config per gripper design
```

## Scoop instead of a gripper?

If the new end effector is a cup or scoop that the ball flies out of, set `release_mode: none`. The arm
then runs the same motion without opening anything; the ball leaves when the arm decelerates at the
follow-through, so tune the follow-through pose and `throw_time_s`.

## Team workflow

The repo is the shared memory of what works, so every teammate tunes from the same numbers.

1. `git pull` before a tuning session, so you start from the latest `config.yaml`.
2. Tune on the arm. Results go to `throws.csv`.
3. After the session, commit the tuned `config.yaml`, `throws.csv` and `recorded_poses.yaml`
   together, with the cup and the hit rate in the message, e.g. `front: throw_time 0.26, 7/10`.
   Committing the log with the config means every result can be traced to the settings that produced it.
4. One person on the arm at a time pushes config changes, to avoid merge conflicts in `config.yaml`.
   If two people tune at once, use `--config mine.yaml` and merge the winning values by hand.
5. When a setting is your best so far, tag it: `git tag best-front-v1 && git push --tags`.
   Before the competition, tag the final config (`git tag competition`) so you can always get back to
   it with `git checkout competition -- config.yaml`, even after late experiments.

If the gripper changes, re-record the poses and keep the old config on a tag, since old tuning won't
transfer to a new gripper.
