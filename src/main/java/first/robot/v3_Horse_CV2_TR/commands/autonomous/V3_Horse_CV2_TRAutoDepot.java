package first.robot.v3_Horse_CV2_TR.commands.autonomous;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.Elastic;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;

public class V3_Horse_CV2_TRAutoDepot {
    public static Command getAutoRoutine(SwerveDrive drive) {
        PathPlannerPath DEPOT;

        try {

            DEPOT = PathPlannerPath.fromPathFile("DEPOT");
            return Commands.sequence(
                Commands.runOnce(() -> 
                 V3_Horse_CV2_TRRobotState.resetPose(AllianceFlipUtil.apply(DEPOT.getStartingHolonomicPose().get()))),
                 AutoBuilder.followPath(DEPOT), Commands.runOnce(
                    () ->
                    drive.stop()
                 )


            );

        } catch (Exception e) {

            e.printStackTrace();
            return Commands.runOnce(
                    () -> Elastic.sendNotification(
                            new Elastic.Notification(
                                    Elastic.NotificationLevel.ERROR,
                                    "Failed to load path",
                                    e.getMessage())))
                    .ignoringDisable(true);
                                
        }

    }
}
