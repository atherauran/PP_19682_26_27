# FTC 19682 — BIOBUZZ

Starter teleop for the 2026–2027 robot, using Pedro Pathing 3.0.2, Ivy 1.1.1,
and FTC SDK 12.0.0. Select **RedTeleOp** or **BlueTeleOp** on the Driver Station; they differ only
in target CELLs and relocalize pose.

The code keeps separate drivetrain, intake, shooter, and motor-driven turret
subsystems, owned by `Robot`. Ivy sequences the feeder pulse without blocking
driving or other controls. Shooter velocity uses the REV Hub's encoder control;
there is no FTCLib dependency or copied shot calibration from last season.
The flywheels idle at 200 encoder ticks/s after START. Set
`SHOOTER_IDLE_VELOCITY = 0` in `RobotConstants` (or Panels) to disable idle spin.
Turret auto aim is implemented but disabled until commissioned (see
[Auto aim](#auto-aim)). There is no active autonomous routine; the existing Auto
Template remains disabled.

## Hardware configuration

Names are in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/constants/RobotConfigs.java`.
Configure the actual motor models correctly in the Driver Station configuration,
especially the shooter motors used for encoder velocity control.

| Name | Device | Role |
| --- | --- | --- |
| `lf`, `lb`, `rf`, `rb` | DC motors | Left/right, front/back drivetrain |
| `intake` | DC motor | Belt-linked pickup and transfer rollers |
| `lshoot`, `rshoot` | Encoder-equipped DC motors | Paired shooter motors |
| `turret` | Encoder-equipped DC motor | Turret yaw |
| `hood` | Positional servo | Geared launch-angle adjustment |
| `feeder` | Positional servo | Transfer-pocket paddle |
| `pp` | goBILDA Pinpoint, I2C | Localization |

The starting Pinpoint configuration assumes goBILDA 4-bar pods. Pod offsets are
currently zero placeholders. Set the actual pod type, directions, and offsets in
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`.

## Controls

| Gamepad | Input | Action |
| --- | --- | --- |
| 1 | Left stick / right stick X | Translate / turn |
| 1 | Hold left stick button | Slow drive |
| 1 | Options | Toggle robot/field-centric driving; starts robot-centric |
| 1 | Start | Relocalize to the alliance LOADING ZONE pose; also sets field-centric forward to the robot's current facing. Use while stationary |
| 1 | X | Toggle auto aim (only when `AUTO_AIM_ENABLED`) |
| 1 | Y | Toggle shooting while moving; starts off |
| 1 | Right trigger | Proportional collection; returns shooter to idle |
| 1 | Left trigger | Proportional ejection; returns shooter to idle; wins over collection |
| 1 | Left bumper | Toggle shooting speed / idle; starts at idle |
| 1 | Tap right bumper | One feeder pulse while shooter is on; intake advances during the pulse |
| 2 | Right stick X | Manual turret rotation while auto aim is off; release to stop |
| 2 | D-pad up/down | Change shooter target by 50 encoder ticks/s; with auto aim, adjusts a correction to the shot table |
| 2 | D-pad left/right | Auto-aim turret correction +1° / −1° |
| 2 | Y / A | Increase/decrease hood servo position by 0.01; with auto aim, adjusts a correction to the shot table |
| 2 | X | Switch target CELL; press after each HIVE TIP |
| 2 | Start | Zero turret encoder at its current position; does not home it |
| 1 or 2 | Hold B | Stop intake, shooter, and turret; return feeder to rest |

Feeding is driver-commanded: `At speed` is telemetry, not an automatic feed
interlock. Wait for both shooter motors to reach the target before feeding.
Collection, ejection, returning the shooter to idle, or B cancels a feeder pulse.
Idle speed does not enable feeding or the `At speed` indicator. Holding B
commands zero flywheel power; releasing it resumes idle. Hold B does not prevent
driving. Use Driver Station STOP to stop the OpMode and all motors.

## Optional flywheel PIDF

Default motor PIDF remains active. To try custom values, uncomment the two
`setVelocityPIDFCoefficients` lines in `Shooter.init()`, then restart the OpMode.
The starting values in `RobotConstants` are P `1.26`, I `0.126`, D `0`, F `12.6`.
These assume about 2600 encoder ticks/s maximum mechanism speed; they are not
measured tuning for this robot. Recalculate from measured maximum speed and
check recovery after shots before relying on them. The starting calculation
follows the [FIRST Global Motor PIDF Tuning Guide](https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/mobilebasic).

## Real-robot commissioning

Settings are in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/constants/RobotConstants.java`
and exposed through Panels. Values here are bench starting points, not measured
mechanical limits or calibrated shooting settings.

1. **Check the configuration and motor directions with no balls.** Match all
   hardware names and motor types. With the drivetrain lifted, verify forward,
   strafe, and turn. Check intake/eject directions, then check that the two
   shooter motors assist each other and launch in the intended direction.
   Mechanism directions live in `RobotConfigs`; drive directions live in Pedro's
   `Constants`. Defaults are 60% drive power, 70% intake power, and 800 shooter
   encoder ticks/s. Lower these for the first direction checks as needed.

2. **Calibrate the two servos before attaching their horns/linkages.** Use
   **Servo Position Test** as described below. In RedTeleOp/BlueTeleOp, INIT sends
   no hood or feeder position commands. START commands hood `0.5` and feeder
   rest `0.5`. Provisional feeder feed is `0.6`, pulse duration is `250 ms`, and
   hood bounds are `0.25–0.75`. Establish the actual rest/feed positions and
   hood travel without binding, update the constants, then reconnect and test
   a single feed cycle. Adjust pulse time to complete the paddle stroke.

3. **Measure the turret's cable-safe range.** Face it forward before INIT; INIT
   resets its encoder without moving it. Manual power starts at `0.15`.
   Software limits are **off**, so reduced power does not prevent cable wrap.
   Jog both ways, record the telemetry ticks before cables tighten or anything
   collides, and leave a margin for stopping. Set `TURRET_MIN_TICKS` and
   `TURRET_MAX_TICKS`, then enable `TURRET_LIMITS_ENABLED`. Verify that each end
   blocks outward travel and still allows retreat. These are power cutoffs,
   not braking-distance compensation. Re-zero only when physically centered.

4. **Set up Pinpoint before using field-centric driving.** Keep the robot still
   during INIT/IMU calibration. Run the existing Pinpoint tuner and copy its
   pod configuration into Pedro's `Constants`. From a zero heading, pushing
   forward should increase X, pushing left should increase Y, and a counter-
   clockwise turn should increase heading. Check a measured translation and
   full rotation, then test field-centric driving. Foresight tuning is not
   needed for this teleop.

5. **Test one ball, then a short sequence.** Confirm the hood direction and
   trajectory, tune shooter speed, and watch both measured velocities recover
   after each shot. Speed is in motor encoder ticks/s, not RPM. Test both ball
   sizes the robot is intended to handle. Check collection/ejection, feeder
   cancellation, B, and Driver Station STOP. On exit, all eight DC motors are
   commanded to zero; servo shutdown is left to the SDK without a homing move.

## Auto aim

Ported from last season's `AutoAimSubsystem_MK2` / `AA_TeleOp_Base`.
`AutoAim` drives the turret toward the upward CELL of our HIVE using odometry,
with feedforward + PID, voltage compensation, and braking prediction. While
aiming, flywheel speed and hood come from the shot table in `RobotConstants`.
Gamepad 2 corrections are added to the shot table. Turret correction is stored
in either mode, but only aiming uses it. With auto aim off, the turret, speed,
and hood are manual as before. `AUTO_AIM_ENABLED` is `false`, so gamepad 1 X
does nothing until it is set in `RobotConstants` or Panels.

Field coordinates: +X away from the audience, +Y toward the red side, HIVE
pivot at (72, 72). Red HIVE CELLs are at Y 84.75, blue at Y 59.25. Upward
CELL aim points are at X 64 (audience side) or 80 (far side). These are
estimates from manual Figures 9-9/9-10, set in `RedTeleOp` / `BlueTeleOp`.
Red starts with the audience CELL up, and blue starts with the far CELL up.
Press gamepad 2 X after every TIP; telemetry shows the current target CELL.

TeleOp starts from the pose saved by autonomous (`autoEndPose`), which is (0, 0, 0)
if no auto ran. Pressing gamepad 1 Start sets the alliance LOADING ZONE pose.
That assumes the robot is centered in it with its back against the wall. Use
`RESET_*` in each TeleOp, which assume an 18 in robot. The robot's heading at
that moment becomes field-centric forward.

Commissioning, in order:

1. **Turret direction.** Positive ticks must be counterclockwise seen from
   above, which turns the turret left. Gamepad 2 stick right should therefore
   decrease ticks. If it doesn't, flip `TURRET_DIRECTION`.
2. **Turret ratio.** Zero at center, jog to a large measured angle, and set
   `TURRET_FULL_RANGE_DEGREE` / `TURRET_FULL_RANGE_ENCODER` to that angle and
   tick count. The current values are placeholders. Check that telemetry
   `Turret (deg)` matches.
3. **Turret limits.** Set `TURRET_MIN_TICKS` / `TURRET_MAX_TICKS` as in
   commissioning step 3. Auto aim only targets angles inside them. A target that
   is out of range even going the long way returns the turret to 0. With both
   limits at 0, auto aim just holds the turret at 0.
4. **Turret offset.** Set `TURRET_OFFSET_X` / `TURRET_OFFSET_Y` to the turret
   axis position relative to the robot center, from CAD.
5. **Controller.** Enable auto aim and tune `AutoAim.TurretController` in
   Panels: `kS`, `kV`, `kA`, then `kP`/`kD`, then `kLinearBraking`. The values
   are last season's and were tuned for a different turret.
6. **Shot table.** `Distance (in)` is shown even with auto aim off. Shoot
   manually from several distances and record working speed and hood values in
   `SHOT_DISTANCES` / `SHOT_VELOCITIES` / `SHOT_HOODS`. Values are interpolated
   linearly and held at the ends. Adjust CELL aim points if shots consistently
   land off-center.
7. **Shooting while moving.** `SOTM_TIME_PER_INCH` / `SOTM_BASE_TIME` are last
   season's lead-time values. Retune them after the shot table.

## Servo position test

Select **Servo Position Test** on the Driver Station. It only uses `hood` and
`feeder`; motors and Pinpoint are not initialized. Disconnect the servo linkages
for the first test. Neither INIT nor START sends a servo position command.

- D-pad left/right selects hood/feeder.
- D-pad up/down adjusts the displayed target by `0.01`; hold the left bumper
  for `0.001` steps.
- Tap **A** to move the selected servo to that target. Adjusting the target or
  switching servos does not move either servo.

Targets start at `HOOD_START` and `FEEDER_REST`. The test allows the full `0–1`
servo range so you can establish limits; it does not use the provisional hood
bounds. Watch for binding as you reconnect and check each mechanism. Telemetry
shows requested positions, not measured shaft positions. Record useful values
and copy them into `RobotConstants`: `HOOD_START`, `HOOD_MIN`, `HOOD_MAX`,
`FEEDER_REST`, and `FEEDER_FEED`. Test values are not saved automatically.

## Build

```sh
./gradlew :TeamCode:assembleDebug
```

APK: `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`.
