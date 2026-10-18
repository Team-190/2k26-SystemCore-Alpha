# GompeiLib Angle Standards: `Angle` vs. `Rotation2d`

This document defines when an angle is a WPILib `Angle` measure and when it is a `Rotation2d`. It applies to GompeiLib and to every robot project built on it. Reviewers can point at it the same way they point at [TESTING.md](TESTING.md).

## Why it matters

A `Rotation2d` stores only a sine and a cosine, and `getRadians()` is `atan2(sin, cos)`, so every `Rotation2d` wraps to (-π, π]. That means a `Rotation2d`:

- can't represent a mechanism that travels more than half a turn in either direction (a continuous arm, a turret, a multi-turn rotary mechanism),
- can't represent a travel limit or a step larger than 180°,
- turns a "position error" into the wrapped difference instead of the distance the mechanism actually has to move,
- throws away the turn count of a multi-turn motor position.

## 1. The rule

| Use | For | Examples |
| --- | --- | --- |
| **`Angle`** (the default) | Any scalar angle: how far something has turned. | Mechanism positions, goals, setpoints, and errors; travel limits, tolerances, offsets, and tuning steps; raw sensor readings (motor and encoder positions, gyro yaw, pitch, and roll); anything that can pass ±180° or where the difference between two values matters. |
| **`Rotation2d`** | An orientation in the 2D plane (field or robot frame) that feeds WPILib geometry or kinematics, where wrapping is correct. | Robot heading, `Pose2d`/`Transform2d` rotation, an aim or auto-align target heading, a swerve wheel angle in `SwerveModuleState`/`SwerveModulePosition`. |
| **`Rotation3d`** | 3D orientation, by the same logic. | Camera mounts, `Pose3d`. |
| **Never** | A raw `double` of radians, degrees, or rotations, or a unit baked into a name. | ~~`yawVelocityRadPerSec`~~, ~~`turnVelocityRadiansPerSecond`~~ |

**Ask the question when it's unclear:** "Is this which way something points in the plane, or how far something has turned?" Which way something points is a `Rotation2d`. How far something has turned is an `Angle`.

## 2. The IO layer is all `Measure`s

Every IO inputs field, IO method parameter, and IO return value is a `Measure` (`Angle` for angles), with no exceptions for values that end up as an orientation. No `Rotation2d` appears in an IO interface, inputs class, or IO implementation.

`Rotation2d` only appears at the subsystem level or later, where the code knows what the value means. The code that turns a reading into geometry does the conversion once, explicitly:

- `SwerveDrive` builds the robot heading `Rotation2d` from the gyro's yaw `Angle`.
- `SwerveModule` builds the wheel `Rotation2d` from the turn motor's `Angle`, and converts its turn goal back to an `Angle` when it calls into the IO.

## 3. Leaving the IO doesn't mean converting

Most subsystems never need a `Rotation2d`. A hood, arm, turret, flywheel, or roller keeps its angles as `Angle` end to end: IO, subsystem, commands, and robot code. Only a subsystem whose angle really is a plane orientation used in geometry or kinematics converts, like the drive or a swerve module.

If a subsystem would convert to `Rotation2d` and then straight back to an `Angle` for its own logic, it should have stayed an `Angle`.

## 4. Worked examples

- **Swerve module.** The wheel angle is a `Rotation2d` in `SwerveModule`: it's an orientation that feeds module states and kinematics, and the turn controller wraps continuously, so a wrapped turn error is the correct shortest-path error. Below it, the turn `MotorIO` and `AbsoluteEncoderIO` report and take `Angle`.
- **Hood.** Hood angles are `Angle` everywhere: the shot lookup tables, `FixedShotParameters.hoodAngle`, `setHoodAngle`/`getHoodAngle`, and the `ShotCalculator` output. The robot headings next to them (`aimAngle`, the chassis angles) are `Rotation2d`.
- **Turret aiming at a field target.** The turret's position and goal are `Angle`. Convert the field-relative target `Rotation2d` into the turret's `Angle` at the point where the aiming math happens.

## 5. Tests

- Each mechanism that stores angles shows that a position past ±180° (or multi-turn, for continuous mechanisms) round-trips through inputs, goals, and errors without wrapping, and that a goal on the far side of ±180° produces the full-length error.
- Each `Angle` to `Rotation2d` boundary conversion is covered once, at that boundary.
