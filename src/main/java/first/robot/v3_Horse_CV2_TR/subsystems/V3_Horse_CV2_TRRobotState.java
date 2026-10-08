package first.robot.v3_Horse_CV2_TR.subsystems;

import edu.wpi.team190.gompeilib.core.state.localization.FieldZone;
import edu.wpi.team190.gompeilib.core.state.localization.Localization;
import first.robot.FieldConstants;
import java.util.HashSet;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.system.Timer;

public class V3_Horse_CV2_TRRobotState {

  private static final Localization localization;

  private static final FieldZone globalZone;
  private static final FieldZone blueHubZone;
  private static final FieldZone redHubZone;
  private static final FieldZone blueTowerZone;
  private static final FieldZone redTowerZone;

  static {
    globalZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.globalTags));
    blueHubZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.blueHubTags));
    redHubZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.redHubTags));
    blueTowerZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.blueTowerTags));
    redTowerZone = new FieldZone(new HashSet<>(FieldConstants.AprilTags.redTowerTags));

    localization =
        new Localization(
            List.of(globalZone, blueHubZone, redHubZone, blueTowerZone, redTowerZone),
            V3_Horse_CV2_TRConstants.DRIVE_CONSTANTS.driveConfig.kinematics(),
            2);
  }

  public static Pose2d getGlobalPose() {
    return localization.getEstimatedPose(globalZone);
  }

  public static void addOdometryObservation(
      Rotation2d rawHeading, SwerveModulePosition[] modulePositions) {
    localization.addOdometryObservation(Timer.getTimestamp(), rawHeading, modulePositions);
  }
}
