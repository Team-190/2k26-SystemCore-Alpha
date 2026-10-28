package first.robot.v3_Horse.commands.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.util.Elastic;
import first.robot.v3_Horse.V3_HorseRobotState;
import first.robot.v3_Horse.commands.V3_HorseCompositeCommands;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntake;
import first.robot.v3_Horse.subsystems.rollerfloor.V3_HorseRollerFloor;
import first.robot.v3_Horse.subsystems.shooter.V3_HorseShooter;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;

public class V3_HorseAutoBackUpShoot {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      V3_HorseShooter shooter,
      V3_HorseRollerFloor rollerFloor,
      V3_HorseIntake intake) {

    PathPlannerPath BACK_UP_SHOOT;
    try {
      BACK_UP_SHOOT = PathPlannerPath.fromPathFile("BACK_UP_SHOOT");

      return Commands.sequence(
          Commands.runOnce(
              () ->
                  V3_HorseRobotState.resetPose(
                      AllianceFlipUtil.apply(BACK_UP_SHOOT.getStartingHolonomicPose().get()))),
          AutoBuilder.followPath(BACK_UP_SHOOT),
          drive.runOnce(drive::stop),
          V3_HorseCompositeCommands.autoScoreOrFeed(drive, rollerFloor, shooter, intake, 10),
          V3_HorseCompositeCommands.stopShooter(rollerFloor, shooter));

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

  public static Pose2d[] getTrajectory() {
    try {
      return PathPlannerPath.fromPathFile("BACK_UP_SHOOT").getPathPoses().toArray(new Pose2d[0]);
    } catch (Exception e) {
      return new Pose2d[0];
    }
  }
}
