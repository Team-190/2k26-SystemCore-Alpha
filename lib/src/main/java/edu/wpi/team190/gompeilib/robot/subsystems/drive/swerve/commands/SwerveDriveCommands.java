package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.sysid.KSCharacterization;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;

/** Robot-agnostic swerve drive commands. */
public final class SwerveDriveCommands {
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private SwerveDriveCommands() {}

  /**
   * Starts a joystick drive. Bind behaviors with {@link JoystickDrive.Builder#with}, then {@link
   * JoystickDrive.Builder#build}.
   *
   * @param drive the drive
   * @param constants the drive's constants, for the driver deadband
   * @param sticks the driver's sticks
   * @param heading the robot's field heading
   */
  public static JoystickDrive.Builder joystickDrive(
      SwerveDrive drive,
      SwerveDriveConstants constants,
      DriveSticks sticks,
      Supplier<Rotation2d> heading) {
    return new JoystickDrive.Builder(drive, sticks, constants.driverDeadband, heading);
  }

  /** Turns the robot in place to a field heading, holding it there until interrupted. */
  public static Command rotateToAngle(
      SwerveDrive drive,
      SwerveDriveConstants constants,
      Supplier<Rotation2d> heading,
      Supplier<Rotation2d> target) {
    return joystickDrive(drive, constants, DriveSticks.NONE, heading)
        .with(() -> true, DriveBehaviors.faceHeading(constants, target))
        .build();
  }

  /** Stops the drive with its modules in an X, holding until interrupted. */
  public static Command stop(SwerveDrive drive) {
    return Commands.run(drive::stopWithX, drive);
  }

  /**
   * Measures the drive motors' static friction in the unit their closed loop outputs, so the result
   * is the drive's kS: torque current for a current closed loop, voltage otherwise.
   */
  public static Command feedforwardCharacterization(
      SwerveDrive drive, SwerveDriveConstants constants) {
    Supplier<AngularVelocity> velocity =
        () -> RotationsPerSecond.of(drive.getFFCharacterizationVelocity());
    if (constants.driveConfig.driveClosedLoopOutputType() == OutputType.CURRENT) {
      return KSCharacterization.current(
          drive,
          current ->
              drive.runCharacterization(
                  new OpenLoopRequest.CurrentOutput(Amps.of(current.in(Amps)))),
          velocity);
    }
    return KSCharacterization.voltage(
        drive,
        voltage ->
            drive.runCharacterization(
                new OpenLoopRequest.VoltageOutput(Volts.of(voltage.in(Volts)))),
        velocity);
  }

  /**
   * Spins the robot in place and, when interrupted, prints the wheel radius that matches the gyro's
   * rotation to the wheels' travel.
   */
  public static Command wheelRadiusCharacterization(
      SwerveDrive drive, SwerveDriveConstants constants) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () ->
                    drive.runVelocity(
                        new ChassisVelocities(
                            0.0, 0.0, limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY))),
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRawGyroRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      Rotation2d rotation = drive.getRawGyroRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () ->
                        printWheelRadius(state, drive, constants.driveConfig.driveBaseRadius()))));
  }

  private static void printWheelRadius(
      WheelRadiusCharacterizationState state, SwerveDrive drive, double driveBaseRadius) {
    double[] positions = drive.getWheelRadiusCharacterizationPositions();
    double wheelDelta = 0.0;
    for (int i = 0; i < 4; i++) {
      wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
    }
    double wheelRadius = (state.gyroDelta * driveBaseRadius) / wheelDelta;

    NumberFormat formatter = new DecimalFormat("#0.000");
    System.out.println("********** Wheel Radius Characterization Results **********");
    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
    System.out.println(
        "\tWheel Radius: "
            + formatter.format(wheelRadius)
            + " meters, "
            + formatter.format(Units.metersToInches(wheelRadius))
            + " inches");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.ZERO;
    double gyroDelta = 0.0;
  }
}
