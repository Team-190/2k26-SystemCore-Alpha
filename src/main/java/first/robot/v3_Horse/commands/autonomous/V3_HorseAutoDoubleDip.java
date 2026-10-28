package first.robot.v3_Horse.commands.autonomous;

import static org.wpilib.units.Units.RadiansPerSecond;

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
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;

public class V3_HorseAutoDoubleDip {
  public static Command getAutoRoutine(
      SwerveDrive drive,
      V3_HorseShooter shooter,
      V3_HorseRollerFloor rollerFloor,
      V3_HorseIntake intake,
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
          V3_HorseCompositeCommands.deployIntake(intake),
          shooter.setFlywheelVelocity(RadiansPerSecond.of(165)),
          Commands.runOnce(
              () ->
                  V3_HorseRobotState.resetPose(
                      AllianceFlipUtil.apply(DOUBLE_DIP_1.getStartingHolonomicPose().get()))),
          AutoBuilder.followPath(DOUBLE_DIP_1),
          drive.runOnce(drive::stop),
          V3_HorseCompositeCommands.autoScoreOrFeed(
              drive, rollerFloor, shooter, intake, shot1Seconds),
          V3_HorseCompositeCommands.stopShooter(rollerFloor, shooter),
          V3_HorseCompositeCommands.deployIntake(intake),
          shooter.setFlywheelVelocity(RadiansPerSecond.of(165)),
          AutoBuilder.followPath(DOUBLE_DIP_2),
          drive.runOnce(drive::stop),
          V3_HorseCompositeCommands.autoScoreOrFeed(
              drive, rollerFloor, shooter, intake, shot2Seconds),
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

  public static Pose2d[] getTrajectory(String path1Name, String path2Name, boolean mirror) {
    try {
      PathPlannerPath DOUBLE_DIP_1 =
          mirror
              ? PathPlannerPath.fromPathFile(path1Name).mirrorPath()
              : PathPlannerPath.fromPathFile(path1Name);
      PathPlannerPath DOUBLE_DIP_2 =
          mirror
              ? PathPlannerPath.fromPathFile(path2Name).mirrorPath()
              : PathPlannerPath.fromPathFile(path2Name);

      List<Pose2d> poses = new ArrayList<>(DOUBLE_DIP_1.getPathPoses());
      poses.addAll(DOUBLE_DIP_2.getPathPoses());
      return poses.toArray(new Pose2d[0]);
    } catch (Exception e) {
      return new Pose2d[0];
    }
  }
}
