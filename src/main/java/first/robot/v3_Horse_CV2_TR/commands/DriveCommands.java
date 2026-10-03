package first.robot.v3_Horse_CV2_TR.commands;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import first.robot.util.AllianceFlipUtil;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;
import org.wpilib.util.Pair;

public final class DriveCommands {

  @Setter private static double lastCardinalDirection = 0.0;
  @Getter private static double slowFactor = 0.5;

  /**
   * A command that drives a SwerveDrive using joystick input.
   *
   * @param drive The SwerveDrive to control.
   * @param driveConstants The constants for the SwerveDrive.
   * @param xSupplier The supplier of the x-axis joystick input.
   * @param ySupplier The supplier of the y-axis joystick input.
   * @param omegaSupplier The supplier of the omega joystick input.
   * @param rotationSupplier The supplier of the rotation of the robot.
   * @param hijackXSuppliers A list of pairs of boolean suppliers and double suppliers for hijacking
   *     the x velocity. If the boolean supplier is true, the x velocity will be set to the value of
   *     the double supplier. If multiple boolean suppliers are true, the first one in the list will
   *     take precedence.
   * @param hijackYSuppliers A list of pairs of boolean suppliers and double suppliers for hijacking
   *     the y velocity. If the boolean supplier is true, the y velocity will be set to the value of
   *     the double supplier. If multiple boolean suppliers are true, the first one in the list will
   *     take precedence.
   * @param hijackOmegaSuppliers A list of pairs of boolean suppliers and double suppliers for
   *     hijacking the omega velocity. If the boolean supplier is true, the omega velocity will be
   *     set to the value of the double supplier. If multiple boolean suppliers are true, the first
   *     one in the list will take precedence.
   * @return A command that drives the SwerveDrive.
   */
  @Trace
  public static Command joystickDrive(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier,
      List<Pair<BooleanSupplier, DoubleSupplier>> hijackXSuppliers,
      List<Pair<BooleanSupplier, DoubleSupplier>> hijackYSuppliers,
      List<Pair<BooleanSupplier, DoubleSupplier>> hijackOmegaSuppliers,
      BooleanSupplier slowMode,
      DoubleSupplier slowFactor) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  driveConstants.driverDeadband);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), driveConstants.driverDeadband);
          linearMagnitude *= linearMagnitude;

          // Calculate new linear velocities

          double fieldRelativeXVel =
              linearMagnitude * linearDirection.getCos() * drive.getMaxLinearSpeedMetersPerSec();
          double fieldRelativeYVel =
              linearMagnitude * linearDirection.getSin() * drive.getMaxLinearSpeedMetersPerSec();

          double angular = omega * drive.getMaxAngularSpeedRadPerSec();

          fieldRelativeXVel =
              hijackXSuppliers.stream()
                  .filter(pair -> pair.getFirst().getAsBoolean())
                  .map(pair -> pair.getSecond().getAsDouble())
                  .findFirst()
                  .orElse(
                      slowMode.getAsBoolean()
                          ? (fieldRelativeXVel * slowFactor.getAsDouble())
                          : fieldRelativeXVel);

          fieldRelativeYVel =
              hijackYSuppliers.stream()
                  .filter(pair -> pair.getFirst().getAsBoolean())
                  .map(pair -> pair.getSecond().getAsDouble())
                  .findFirst()
                  .orElse(
                      slowMode.getAsBoolean()
                          ? (fieldRelativeYVel * slowFactor.getAsDouble())
                          : fieldRelativeYVel);

          angular =
              hijackOmegaSuppliers.stream()
                  .filter(pair -> pair.getFirst().getAsBoolean())
                  .map(pair -> pair.getSecond().getAsDouble())
                  .findFirst()
                  .orElse(slowMode.getAsBoolean() ? (angular * slowFactor.getAsDouble()) : angular);

          ChassisVelocities chassisSpeeds =
              new ChassisVelocities(fieldRelativeXVel, fieldRelativeYVel, angular)
                  .toFieldRelative(AllianceFlipUtil.apply(rotationSupplier.get()));

          Logger.recordOutput("Drive/JoystickDrive/chassisSpeeds", chassisSpeeds);

          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  public static Command joystickDrive(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier) {
    return joystickDrive(
        drive,
        driveConstants,
        xSupplier,
        ySupplier,
        omegaSupplier,
        rotationSupplier,
        List.of(),
        List.of(),
        List.of(),
        () -> false,
        () -> 1);
  }
}
