package first.robot.v3_Horse_CV2_TR.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.Elastic;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import first.robot.v3_Horse_CV2_TR.commands.V3_Horse_CV2_TRCompositeCommands;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_Intake;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooter;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class V3_Horse_CV2_TRAutoLeftOutliers {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      V3_Horse_CV2_TRShooter shooter,
      V3_Horse_CV2_TRRollerFloor rollerFloor,
      V3_Horse_CV2_TR_Intake intake) {

    PathPlannerPath DOUBLE_DIP_1;
    PathPlannerPath DOUBLE_DIP_2;

    try {
      DOUBLE_DIP_1 = PathPlannerPath.fromPathFile("DOUBLE_DIP_1");
      DOUBLE_DIP_2 = PathPlannerPath.fromPathFile("DOUBLE_DIP_2_OUTLIERS");

      return Commands.sequence(
          V3_Horse_CV2_TRCompositeCommands.deployIntake(intake),
          Commands.runOnce(
              () ->
                  V3_Horse_CV2_TRRobotState.resetPose(
                      AllianceFlipUtil.apply(DOUBLE_DIP_1.getStartingHolonomicPose().get()))),
          AutoBuilder.followPath(DOUBLE_DIP_1),
          drive.runOnce(drive::stop),
          V3_Horse_CV2_TRCompositeCommands.autoScoreOrFeed(drive, rollerFloor, shooter, intake, 7),
          V3_Horse_CV2_TRCompositeCommands.stopShooter(rollerFloor, shooter),
          Commands.waitSeconds(5), // for the inertia wheel to calm down
          V3_Horse_CV2_TRCompositeCommands.deployIntake(intake),
          AutoBuilder.followPath(DOUBLE_DIP_2),
          V3_Horse_CV2_TRCompositeCommands.autoScoreOrFeed(drive, rollerFloor, shooter, intake, 5));
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
