package first.robot.v3_Horse.commands;

import static org.wpilib.units.Units.MetersPerSecondPerSecond;

import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants.AutoAlignConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.SwerveDriveCommands;
import first.robot.FieldConstants;
import first.robot.v3_Horse.V3_HorseRobotState;
import java.util.function.Supplier;
import lombok.Getter;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;

/**
 * This robot's drive commands, built on GompeiLib's {@link SwerveDriveCommands}. Joystick driving
 * is bound in the robot container.
 */
public final class V3_HorseDriveCommands {

  @Getter private static double slowFactor = 0.5;

  public static Command incrementSlowFactor() {
    return Commands.runOnce(() -> slowFactor = Math.min(slowFactor + 0.05, 1.0));
  }

  public static Command decrementSlowFactor() {
    return Commands.runOnce(() -> slowFactor = Math.max(slowFactor - 0.05, 0.0));
  }

  public static boolean atAngle(Rotation2d currentRotation, Rotation2d targetRotation) {
    return Math.abs(currentRotation.minus(targetRotation).getRadians())
        <= Units.degreesToRadians(2.0);
  }

  public static Command autoAlignPoseCommand(
      SwerveDrive drive,
      Supplier<Pose2d> robotPoseSupplier,
      Pose2d targetPose,
      AutoAlignConstants constants) {
    return new V3_HorseAutoAlignCommands(
        drive,
        targetPose,
        () -> true,
        robotPoseSupplier,
        constants,
        // Must be finite: alpha-7 TrapezoidProfile computes inf * 0 = NaN otherwise
        constants.xConstraints().maxAcceleration().get().in(MetersPerSecondPerSecond));
  }

  public static Command autoAlignTowerCommand(
      SwerveDrive drive, Supplier<Pose2d> robotPoseSupplier, AutoAlignConstants constants) {
    return autoAlignPoseCommand(
        drive,
        robotPoseSupplier,
        new Pose2d(FieldConstants.Tower.centerPoint, new Rotation2d()),
        constants);
  }

  /** Turns in place so the shooter faces the hub or the nearest feed point. */
  public static Command aimAtHub(SwerveDrive drive, SwerveDriveConstants driveConstants) {
    return SwerveDriveCommands.rotateToAngle(
        drive, driveConstants, V3_HorseRobotState::getHeading, V3_HorseRobotState::getAimAngle);
  }
}
