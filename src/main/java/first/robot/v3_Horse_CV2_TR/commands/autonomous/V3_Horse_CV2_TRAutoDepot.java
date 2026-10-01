package first.robot.v3_Horse_CV2_TR.commands.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.Elastic;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class V3_Horse_CV2_TRAutoDepot {
  public static Command getAutoRoutine(SwerveDrive drive) {
    PathPlannerPath DEPOT;
    try {
      DEPOT = PathPlannerPath.fromPathFile("DEPOT");
      return Commands.sequence(
          Commands.runOnce(
              () ->
                  V3_Horse_CV2_TRRobotState.resetPose(
                      AllianceFlipUtil.apply(DEPOT.getStartingHolonomicPose().get()))));

    } catch (Exception e) {
      e.printStackTrace();
      return Commands.runOnce(
              () ->
                  Elastic.sendNotification(
                      new Elastic.Notification(
                          Elastic.NotificationLevel.ERROR, "Failed to load path", e.getMessage())))
          .ignoringDisable(true);
    }
  }
}
