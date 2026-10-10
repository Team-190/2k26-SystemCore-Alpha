package first.robot.v3_Horse_CV2_TR.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.Elastic;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import first.robot.v3_Horse_CV2_TR.commands.V3_Horse_CV2_TRCompositeCommands;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TRRollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooter;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class V3_Horse_CV2_TRAutoBackUpShoot {
  public static Command getAutoRoutine(
      SwerveDrive drive, V3_Horse_CV2_TRShooter shooter, V3_Horse_CV2_TRRollerFloor rollerFloor) {

    PathPlannerPath BACK_UP_SHOOT;
    try {
      BACK_UP_SHOOT = PathPlannerPath.fromPathFile("BACK_UP_SHOOT");

      return Commands.sequence(
          Commands.runOnce(
              () ->
                  V3_Horse_CV2_TRRobotState.resetPose(
                      AllianceFlipUtil.apply(BACK_UP_SHOOT.getStartingHolonomicPose().get()))),
          AutoBuilder.followPath(BACK_UP_SHOOT),
          drive.runOnce(drive::stop),
          V3_Horse_CV2_TRCompositeCommands.scoreOrFeedCommand(rollerFloor, shooter));
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
