package first.robot.v3_Horse_CV2_TR.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.Elastic;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

public class V3_Horse_CV2_TRAutoLeftCheesy {
  public static Command getAutoRoutine(SwerveDrive drive) {
    PathPlannerPath CHEESY_1, CHEESY_2;
    try {
      CHEESY_1 = PathPlannerPath.fromPathFile("CHEESY_1");
      CHEESY_2 = PathPlannerPath.fromPathFile("CHEESY_2");
      return Commands.sequence(
          Commands.runOnce(
              () ->
                  V3_Horse_CV2_TRRobotState.resetPose(
                      AllianceFlipUtil.apply(CHEESY_1.getStartingHolonomicPose().get()))),
          AutoBuilder.followPath(CHEESY_1),
          Commands.waitSeconds(0),
          AutoBuilder.followPath(CHEESY_2),
          Commands.runOnce(() -> drive.stop()));
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
