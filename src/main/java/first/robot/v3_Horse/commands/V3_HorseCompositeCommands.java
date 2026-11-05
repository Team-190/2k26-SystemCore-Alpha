package first.robot.v3_Horse.commands;

import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.v3_Horse.V3_HorseConstants;
import first.robot.v3_Horse.V3_HorseRobotState;
import first.robot.v3_Horse.V3_HorseRobotState.FixedShots;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntake;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.ExtensionState;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.RollerState;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloor;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloorConstants;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloorConstants.RollerFloorState;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooter;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooterConstants.ShooterGoal;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class V3_HorseCompositeCommands {

  public static Command resetHeading(
      SwerveDrive drive,
      Consumer<Pose2d> resetHeadingConsumer,
      Supplier<Translation2d> currentRobotTranslation) {
    return Commands.runOnce(
            () -> {
              resetHeadingConsumer.accept(
                  new Pose2d(
                      currentRobotTranslation.get(), AllianceFlipUtil.apply(new Rotation2d())));
            })
        .ignoringDisable(true);
  }

  public static Command updateCurrentLimits(
      SwerveDrive drive, double driveCurrentLimit, double turnCurrentLimit) {
    return Commands.runOnce(() -> drive.updateCurrentLimits(driveCurrentLimit, turnCurrentLimit))
        .ignoringDisable(true);
  }

  public static Command deployIntake(V3_HorseIntake intake) {
    return Commands.parallel(
        intake.setExtensionState(ExtensionState.INTAKE), intake.setRollerState(RollerState.IDLE));
  }

  public static Command intakeCollect(V3_HorseIntake intake) {
    return Commands.parallel(
        intake.setExtensionState(ExtensionState.INTAKE), intake.setRollerState(RollerState.INTAKE));
  }

  public static Command intakeStow(V3_HorseIntake intake) {
    return Commands.parallel(
        intake.setExtensionState(ExtensionState.STOW), intake.setRollerState(RollerState.IDLE));
  }

  public static Command intakeSpit(V3_HorseIntake intake, V3_HorseRollerFloor rollerfloor) {
    return Commands.parallel(
        intake.setExtensionState(ExtensionState.INTAKE),
        intake.setRollerState(RollerState.EXTAKE),
        rollerfloor.setState(RollerFloorState.OVERRIDE),
        Commands.runOnce(
            () ->
                rollerfloor.setOverrideRollerFloorVoltage(
                    V3_HorseRollerFloorConstants.ROLLER_FLOOR_RUN_VOLTAGE.times(-1))));
  }

  public static Command autoScoreOrFeed(
      SwerveDrive drive,
      V3_HorseRollerFloor rollerFloor,
      V3_HorseShooter shooter,
      V3_HorseIntake intake,
      double seconds) {
    return Commands.race(
        V3_HorseDriveCommands.aimAtHub(drive, V3_HorseConstants.DRIVE_CONSTANTS),
        scoreOrFeedCommand(rollerFloor, shooter, intake),
        Commands.waitSeconds(seconds));
  }

  /**
   * Spins up for score or feed, thxen runs the roller floor once the shooter and heading are ready.
   */
  public static Command scoreOrFeedCommand(
      V3_HorseRollerFloor rollerFloor, V3_HorseShooter shooter, V3_HorseIntake intake) {
    return Commands.parallel(
        shooter.setGoal(
            () -> (V3_HorseRobotState.isInAllianceZone() ? ShooterGoal.SCORE : ShooterGoal.FEED)),
        Commands.sequence(
            rollerFloor.setState(RollerFloorState.SPIN_UP),
            Commands.waitUntil(() -> shooter.atGoal() && isAimedForShot()),
            rollerFloor.setState(RollerFloorState.RUN),
            intake.squishIntake()));
  }

  private static boolean isAimedForShot() {
    return V3_HorseDriveCommands.atAngle(
        V3_HorseRobotState.getHeading(), V3_HorseRobotState.getAimAngle());
  }

  public static Command stopShooter(V3_HorseRollerFloor rollerFloor, V3_HorseShooter shooter) {
    return Commands.parallel(
        shooter.setGoal(ShooterGoal.STOP), rollerFloor.setState(RollerFloorState.STOP));
  }

  /**
   * Holds the fixed robot heading while spinning up, then runs the roller floor once the shooter
   * and heading are ready.
   */
  public static Command fixedShotCommand(
      SwerveDrive drive,
      V3_HorseRollerFloor rollerFloor,
      V3_HorseShooter shooter,
      V3_HorseIntake intake,
      FixedShots fixedShot) {
    return Commands.parallel(
        Commands.sequence(
            shooter.runFixedShot(fixedShot),
            rollerFloor.setState(RollerFloorState.SPIN_UP),
            Commands.waitUntil(() -> shooter.atGoal()),
            rollerFloor.setState(RollerFloorState.RUN),
            intake.squishIntake()));
  }
}
