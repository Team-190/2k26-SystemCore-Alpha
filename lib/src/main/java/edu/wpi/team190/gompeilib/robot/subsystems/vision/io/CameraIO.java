package edu.wpi.team190.gompeilib.robot.subsystems.vision.io;

import com.limelightvision.FiducialTarget;
import com.limelightvision.IMUMode;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.camera.Camera;
import java.util.Arrays;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;

/**
 * An interface for the hardware implementation of a {@link Camera Camera}. Contains the methods
 * necessary for providing information to a camera, like its pipeline, and getting information out
 * of a camera, like its individual robot pose estimate.
 */
public interface CameraIO {

  @AutoLog
  public static class GompeiVisionIOInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public double captureFPS = 0;
    public double processingFPS = 0;
  }

  @AutoLog
  public static class LimelightIOInputs {
    public PoseEstimate mt1PoseEstimate = new PoseEstimate();
    public PoseEstimate mt2PoseEstimate = new PoseEstimate();
    public RawFiducial[] rawFiducials = {};
  }

  public default void updateInputs(GompeiVisionIOInputs inputs) {}

  public default void updateInputs(LimelightIOInputs inputs) {}

  public default String getName() {
    return "";
  }

  public default void setCameraPose(Pose3d pose) {}

  public default void setRobotOrientation(
      org.wpilib.units.measure.Angle yaw, org.wpilib.units.measure.AngularVelocity yawRate) {
    setRobotOrientation(
        yaw.in(org.wpilib.units.Units.Degrees),
        yawRate.in(org.wpilib.units.Units.DegreesPerSecond));
  }

  public default void setSharedRobotOrientation(
      org.wpilib.units.measure.Angle yaw, org.wpilib.units.measure.AngularVelocity yawRate) {
    setSharedRobotOrientation(
        yaw.in(org.wpilib.units.Units.Degrees),
        yawRate.in(org.wpilib.units.Units.DegreesPerSecond));
  }

  public default void setRobotOrientation(double yawDegrees, double yawRateDegreesPerSec) {}

  public default void setSharedRobotOrientation(double yawDegrees, double yawRateDegreesPerSec) {}

  public default void setThrottle(int throttle) {}

  public default void setIMUMode(IMUMode mode) {}

  public default void setIMUMode(int mode) {
    setIMUMode(
        switch (mode) {
          case 0 -> IMUMode.EXTERNAL;
          case 1 -> IMUMode.EXTERNAL_SEED_INTERNAL;
          case 2 -> IMUMode.INTERNAL;
          case 3 -> IMUMode.INTERNAL_MT1_ASSIST;
          case 4 -> IMUMode.INTERNAL_EXTERNAL_ASSIST;
          default -> IMUMode.UNKNOWN;
        });
  }

  public default void setIMUAssistAlpha(double alpha) {}

  public default void setRewindEnabled(boolean enabled) {}

  public default void triggerRewindCapture(org.wpilib.units.measure.Time duration) {
    triggerRewindCapture(duration.in(org.wpilib.units.Units.Seconds));
  }

  public default void triggerRewindCapture(double durationSeconds) {}

  public default void flush() {}

  public record PoseEstimate(
      Pose2d pose,
      double timestampSeconds,
      double latency,
      int tagCount,
      double tagSpan,
      double avgTagDist,
      double avgTagArea,
      RawFiducial[] rawFiducials,
      boolean isMegaTag2) {
    public PoseEstimate() {
      this(Pose2d.ZERO, 0, 0, 0, 0, 0, 0, new RawFiducial[] {}, false);
    }

    public PoseEstimate(com.limelightvision.PoseEstimate poseEstimate) {
      this(
          poseEstimate != null && poseEstimate.pose != null ? poseEstimate.pose : Pose2d.ZERO,
          poseEstimate != null ? poseEstimate.timestampSeconds : 0.0,
          poseEstimate != null ? poseEstimate.latencyMillis : 0.0,
          poseEstimate != null ? poseEstimate.reportedTagCount : 0,
          poseEstimate != null ? poseEstimate.tagSpanMeters : 0.0,
          poseEstimate != null ? poseEstimate.avgTagDistanceMeters : 0.0,
          poseEstimate != null ? poseEstimate.avgTagAreaPercent : 0.0,
          poseEstimate == null || poseEstimate.rawFiducials == null
              ? new RawFiducial[] {}
              : Arrays.stream(poseEstimate.rawFiducials)
                  .map(RawFiducial::new)
                  .toArray(RawFiducial[]::new),
          poseEstimate != null && poseEstimate.isMT2());
    }
  }

  public record RawFiducial(
      int id,
      double txnc,
      double tync,
      double ta,
      double distToCamera,
      double distToRobot,
      double ambiguity) {
    public RawFiducial() {
      this(0, 0, 0, 0, 0, 0, 0);
    }

    public RawFiducial(FiducialTarget target) {
      this(
          target != null ? target.fiducialId : 0,
          target != null ? target.txDegreesNoCrosshair : 0.0,
          target != null ? target.tyDegreesNoCrosshair : 0.0,
          target != null ? target.targetAreaPercent : 0.0,
          target != null ? target.getDistanceToCamera() : 0.0,
          target != null ? target.getDistanceToRobot() : 0.0,
          target != null ? target.ambiguity : 0.0);
    }
  }

  public record RawDetection(
      int classId,
      double txnc,
      double tync,
      double ta,
      double corner0_X,
      double corner0_Y,
      double corner1_X,
      double corner1_Y,
      double corner2_X,
      double corner2_Y,
      double corner3_X,
      double corner3_Y) {
    public RawDetection() {
      this(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }
  }
}
