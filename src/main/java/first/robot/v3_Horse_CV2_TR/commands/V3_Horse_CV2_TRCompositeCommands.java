package first.robot.v3_Horse_CV2_TR.commands;

import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState.FixedShots;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_Intake;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants.ExtensionState;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants.RollerState;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloorConstants;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloorConstants.RollerFloorState;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooter;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants.ShooterGoal;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

public class V3_Horse_CV2_TRCompositeCommands {

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

  public static Command intakeCollect(V3_Horse_CV2_TR_Intake intake) {
    return Commands.parallel(
        intake.setExtensionState(ExtensionState.INTAKE), intake.setRollerState(RollerState.INTAKE));
  }

  public static Command intakeStow(V3_Horse_CV2_TR_Intake intake) {
    return Commands.parallel(
        intake.setExtensionState(ExtensionState.STOW), intake.setRollerState(RollerState.IDLE));
  }

  public static Command intakeSpit(
      V3_Horse_CV2_TR_Intake intake, V3_Horse_CV2_TRRollerFloor rollerfloor) {
    return Commands.parallel(
        intake.setExtensionState(ExtensionState.INTAKE),
        intake.setRollerState(RollerState.EXTAKE),
        rollerfloor.setState(RollerFloorState.OVERRIDE),
        Commands.runOnce(
            () ->
                rollerfloor.setOverrideRollerFloorVoltage(
                    V3_Horse_CV2_TRRollerFloorConstants.ROLLER_FLOOR_RUN_VOLTAGE.times(-1))));
  }

  /**
   * Spins up for score or feed, then runs the roller floor once the shooter and heading are ready.
   */
  public static Command scoreOrFeedCommand(
      V3_Horse_CV2_TRRollerFloor rollerFloor, V3_Horse_CV2_TRShooter shooter) {
    return Commands.parallel(
        shooter.setGoal(
            () ->
                (V3_Horse_CV2_TRRobotState.isInAllianceZone()
                    ? ShooterGoal.SCORE
                    : ShooterGoal.FEED)),
        Commands.sequence(
            Commands.waitUntil(() -> shooter.atGoal() && isAimedForShot()),
            rollerFloor.setState(RollerFloorState.RUN)));
  }

  private static boolean isAimedForShot() {
    return V3_Horse_CV2_TRDriveCommands.atAngle(
        V3_Horse_CV2_TRRobotState.getHeading(), V3_Horse_CV2_TRRobotState.getAimAngle());
  }

  public static Command stopShooter(
      V3_Horse_CV2_TRRollerFloor rollerFloor, V3_Horse_CV2_TRShooter shooter) {
    return Commands.parallel(
        shooter.setGoal(ShooterGoal.STOP), rollerFloor.setState(RollerFloorState.STOP));
  }

  /**
   * Holds the fixed robot heading while spinning up, then runs the roller floor once the shooter
   * and heading are ready.
   */
  public static Command fixedShotCommand(
      SwerveDrive drive,
      V3_Horse_CV2_TRRollerFloor rollerFloor,
      V3_Horse_CV2_TRShooter shooter,
      FixedShots fixedShot) {
    return Commands.parallel(
        Commands.sequence(
            shooter.runFixedShot(fixedShot),
            Commands.waitUntil(() -> shooter.atGoal()),
            rollerFloor.setState(RollerFloorState.RUN)));
  }
}
