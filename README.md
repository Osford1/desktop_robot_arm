# Desktop robot arm — serial joint control

**A 3-joint arm plus gripper on four steppers, driven from the Arduino IDE's serial monitor.** Type an
angle per joint and it moves there with acceleration-limited motion while the other joints keep
running — no blocking `delay()` in the motion path.

Firmware sketch, not a library: one file, Arduino UNO, CNC Shield V3, AccelStepper.

## Commands

Open the serial monitor at **115200 baud** and type one line per command:

| Command | Effect |
| --- | --- |
| `ALL a b c` | move all three joints at once — `a`, `b`, `c` are joint angles in degrees (`ALL 0 30 -20`) |
| `J1 <deg>` | joint 1 to an angle |
| `J2 <deg>` | joint 2 to an angle — mapped to the A pins (12/13), which some clone shields do not break out |
| `J3 <deg>` | joint 3 to an angle |
| `G <steps>` | gripper move, **relative** to where it is now; negative goes the other way |
| `GOTO <steps>` | gripper move to an **absolute** step position |
| `GINV 0\|1` | invert the gripper direction in software, for when the gripper is assembled mirrored |

Joint angles are converted to steps with the joint's gear ratio; negative angles are accepted.

## Hardware

| Item | What I used |
| --- | --- |
| Board | Arduino UNO |
| Shield | CNC Shield V3 |
| Drivers | 4 × step/dir drivers |
| Motors | 3 × stepper on the joints, 1 × stepper on the gripper |
| Microstepping | 1/8 (`MICROSTEPS = 8`, matched to the shield's MS jumpers) |
| Gearboxes | 25:1 (joint 1), 100:1 (joint 2), 10:1 (joint 3) |

## Pin map

| Axis | Function | STEP | DIR | Shield axis |
| --- | --- | --- | --- | --- |
| Joint 1 | `j1` | 2 | 5 | X |
| Joint 2 | `j2` | 12 | 13 | A |
| Joint 3 | `j3` | 4 | 7 | Z |
| Gripper | `g` | 3 | 6 | Y |
| All drivers | `EN` | 8 | — | enable, active low |

## How the motion works

- **Angle → steps**: `steps = (angle / 360) × 200 × MICROSTEPS × gear_ratio`. One motor revolution is
  200 full steps, so joint 2 at 100:1 needs 160,000 steps per output revolution — fine at a few degrees
  per command, and a reminder that the high-ratio joint is slow.
- **Non-blocking**: the main loop calls `run()` on each `AccelStepper` every iteration, so all four move
  together and new serial commands are parsed while the arm is still travelling. Nothing waits for a
  move to finish.
- **Acceleration-limited**: each motor has its own max speed and acceleration, so a joint does not start
  at full speed.
- **Enable pin** is driven in software so the drivers can be released when idle.

## Knobs

| Constant | File | Default | What it does |
| --- | --- | --- | --- |
| `MOTOR_STEPS` | `joint_control.c` | 200 | full steps per motor revolution (1.8°/step) |
| `MICROSTEPS` | `joint_control.c` | 8 | must match the shield's MS jumpers |
| `GR1`, `GR2`, `GR3` | `joint_control.c` | 25.0, 100.0, 10.0 | gearbox ratio per joint |
| `MAX_SPEED_1..3`, `ACCEL_1..3` | `joint_control.c` | 8000, 1600 | joint speed (steps/s) and acceleration (steps/s²) |
| `MAX_SPEED_G`, `ACCEL_G` | `joint_control.c` | 400, 200 | gripper limits, deliberately slow |
| `X_STEP`/`X_DIR` … | `joint_control.c` | see pin map | pin assignments |
| `EN_ACTIVE_LOW` | `joint_control.c` | `true` | most CNC shields are active low |

## Run it

1. Open `Joint_control/joint_control.c` in the Arduino IDE (as a `.ino` sketch — see the note below).
2. Select Arduino UNO and the right port, upload.
3. Open the serial monitor, set **115200 baud**, and type a command.

> The file is `.c`, but the Arduino IDE only builds `.ino` sketches. Copy or rename it to
> `joint_control.ino` in a folder of the same name before opening it.

## Troubleshooting (errors I actually hit)

| Symptom | Cause | Fix |
| --- | --- | --- |
| Nothing moves but the sketch runs | drivers disabled | check `EN_PIN` wiring and `EN_ACTIVE_LOW` |
| A joint runs backwards | coil pairs swapped | swap that axis's DIR polarity or invert the gear-ratio sign |
| Motion is coarse or fast | microstepping jumpers | set `MICROSTEPS` to the jumper setting |
| Joint travels the wrong distance | wrong gear ratio | count the gearbox stages and fix `GR1`/`GR2`/`GR3` |
| Serial input ignored | baud mismatch | 115200 in the monitor |
| Arm slams into its stop | no soft limits | this sketch has none — command small moves and add limits before leaving it alone |
| Gripper closes instead of opening | assembled mirrored | `GINV 1` inverts it without rewiring |
| A joint only moves once then stops | relative vs absolute confusion | `G` is relative and accumulates; `GOTO` is absolute |

## Status

Worked on the bench: all four axes move, `ALL` and per-joint commands both work, gripper steps respond.
No limits, no homing, no feedback — the drivers are open-loop, so the arm only knows where it is
because it started somewhere known. Treat any published angle as relative to the power-on position.

<!-- TODO(owner): no photo or wiring diagram of the arm in the repo. Add one to docs/screenshots/ and
reference it here. A pin map photo over the CNC shield would help most. -->

## Credits

AccelStepper by Mike McCauley (see [AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/)).
CNC Shield V3 pin conventions follow the common Arduino CNC shield layout.

MIT — see `LICENSE`.
