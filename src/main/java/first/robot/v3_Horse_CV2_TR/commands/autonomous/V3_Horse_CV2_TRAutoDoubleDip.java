package first.robot.v3_Horse_CV2_TR.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.Elastic;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState.FixedShots;
import first.robot.v3_Horse_CV2_TR.commands.V3_Horse_CV2_TRCompositeCommands;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_Intake;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooter;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class V3_Horse_CV2_TRAutoDoubleDip {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      V3_Horse_CV2_TRShooter shooter,
      V3_Horse_CV2_TRRollerFloor rollerFloor,
      V3_Horse_CV2_TR_Intake intake,
      String path1Name,
      String path2Name,
      boolean mirror,
      double shot1Seconds,
      double shot2Seconds) {

    PathPlannerPath DOUBLE_DIP_1;
    PathPlannerPath DOUBLE_DIP_2;

    try {

      DOUBLE_DIP_1 =
          mirror
              ? PathPlannerPath.fromPathFile(path1Name).mirrorPath()
              : PathPlannerPath.fromPathFile(path1Name);
      DOUBLE_DIP_2 =
          mirror
              ? PathPlannerPath.fromPathFile(path2Name).mirrorPath()
              : PathPlannerPath.fromPathFile(path2Name);

      return Commands.sequence(
          V3_Horse_CV2_TRCompositeCommands.deployIntake(intake),
          shooter.setFlywheelVelocity(FixedShots.FRONT_TOWER.getParameters().flywheelSpeed().get()),
          Commands.runOnce(
              () ->
                  V3_Horse_CV2_TRRobotState.resetPose(
                      AllianceFlipUtil.apply(DOUBLE_DIP_1.getStartingHolonomicPose().get()))),
          AutoBuilder.followPath(DOUBLE_DIP_1),
          drive.runOnce(drive::stop),
          V3_Horse_CV2_TRCompositeCommands.autoScoreOrFeed(
              drive, rollerFloor, shooter, intake, shot1Seconds),
          V3_Horse_CV2_TRCompositeCommands.stopShooter(rollerFloor, shooter),
          V3_Horse_CV2_TRCompositeCommands.deployIntake(intake),
          shooter.setFlywheelVelocity(FixedShots.FRONT_TOWER.getParameters().flywheelSpeed().get()),
          AutoBuilder.followPath(DOUBLE_DIP_2),
          drive.runOnce(drive::stop),
          V3_Horse_CV2_TRCompositeCommands.autoScoreOrFeed(
              drive, rollerFloor, shooter, intake, shot2Seconds),
          V3_Horse_CV2_TRCompositeCommands.stopShooter(rollerFloor, shooter));
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.runOnce(
              () ->
                  Elastic.sendNotification(
                      new Elastic.Notification(
                          Elastic.NotificationLevel.ERROR,
                          "Failed to load auto path",
                          e.getMessage())))
          .ignoringDisable(true);
    }
  }
}
