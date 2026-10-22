package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import com.limelightvision.IMUMode;
import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants.LimelightConfig;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionSingleTxTyObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.LimelightIOInputsAutoLogged;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.system.Timer;

/**
 * Abstract base class managing pipeline lifecycle, telemetry, gating, and pose observation dispatch
 * for all Limelight vision cameras (both static and articulated/moving variants).
 *
 * <h3>Pipeline Lifecycle in {@link #periodic()}</h3>
 *
 * <ol>
 *   <li>{@link #updateCameraState()}: Optional subclass hook called before processing (e.g.,
 *       buffering live turret angles).
 *   <li><b>State Transitions:</b> Handles enable/disable transitions: switches IMU modes (LL4:
 *       {@code EXTERNAL_SEED_INTERNAL} while disabled, {@code INTERNAL_EXTERNAL_ASSIST} while
 *       enabled), adjusts processing throttle (low framerate while disabled unless tuning), and
 *       triggers periodic rewind captures.
 *   <li>{@link #publishOrientation()}: Publishes robot or turret heading to Limelight so MegaTag2
 *       can perform gyro disambiguation.
 *   <li><b>IO Input Polling:</b> Pulls updated data via {@code io.updateInputs(inputs)} and logs to
 *       AdvantageKit.
 *   <li><b>MegaTag1 &amp; MegaTag2 Gating:</b> Validates timestamps (stale frame rejection), tag
 *       counts, chassis yaw rates, tag distance, single-tag ambiguity, and field boundary bounds.
 *   <li><b>Robot Pose Resolution:</b> Uses {@link #getCameraPoseAt(double)} and {@link
 *       #resolveRobotPose(Pose2d, Pose3d)} to convert the raw Limelight estimate into the true
 *       field robot pose (accounting for mechanism latency and kinematics in articulated variants).
 *   <li><b>Standard Deviation &amp; Observation Dispatch:</b> Computes dynamic standard deviations
 *       scaled by tag distance and tag count, then dispatches {@link VisionPoseObservation}s and
 *       {@link VisionSingleTxTyObservation}s to registered consumers.
 * </ol>
 */
public abstract class CameraLimelight extends Camera {
  protected final LimelightIOInputsAutoLogged inputs;
  protected final CameraIO io;
  protected final LimelightConfig limelightConfig;
  @Getter protected final String name;

  protected final Supplier<Rotation2d> headingSupplier;
  protected final Supplier<ChassisVelocities> chassisSpeedsSupplier;

  @Getter protected final List<Pose3d> allTagPoses;

  protected boolean wasEnabled;
  protected double enabledTimestamp;

  protected double lastMT1TimestampSeconds = -1.0;
  protected double lastMT2TimestampSeconds = -1.0;

  public CameraLimelight(
      CameraIO io,
      LimelightConfig config,
      Pose3d initialCameraPose,
      Supplier<Rotation2d> headingSupplier,
      Supplier<ChassisVelocities> chassisSpeedsSupplier,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
    super(config.key(), poseObservers, new ArrayList<>(), singleTxTyObservers);

    inputs = new LimelightIOInputsAutoLogged();
    this.io = io;
    this.limelightConfig = config;
    this.name = "limelight-" + config.key();

    this.headingSupplier = headingSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;

    allTagPoses = new ArrayList<>();
    realTimeCameraPose = initialCameraPose;

    io.setCameraPose(realTimeCameraPose);
    io.setRewindEnabled(config.enableRewind());
    if (config.cameraType() != null && config.cameraType().hasIMU()) {
      io.setIMUMode(IMUMode.EXTERNAL_SEED_INTERNAL);
      io.setIMUAssistAlpha(config.imuAssistAlpha() > 0 ? config.imuAssistAlpha() : 0.001);
    } else {
      io.setIMUMode(IMUMode.EXTERNAL);
    }
    io.setThrottle(GompeiLib.isTuning() ? 0 : 190);

    wasEnabled = false;
    enabledTimestamp = Timer.getTimestamp();
  }

  public record ResolvedCapture(Pose2d robotPose, Pose3d cameraPose) {}

  /** Publishes robot heading/orientation to NetworkTables (shared or individual). */
  protected abstract void publishOrientation();

  /** Resolves the field-space robot pose and 3D camera pose for a given capture. */
  protected ResolvedCapture resolveCapture(Pose2d rawReportedRobotPose, double timestampSeconds) {
    return new ResolvedCapture(rawReportedRobotPose, realTimeCameraPose);
  }

  @Override
  @Trace
  public void periodic() {
    poseObservationList.clear();
    multiTxTyObservationList.clear();
    singleTxTyObservationList.clear();

    if (RobotState.isEnabled()) {
      if (!wasEnabled) {
        enabledTimestamp = Timer.getTimestamp();
        wasEnabled = true;
        if (limelightConfig.cameraType() != null && limelightConfig.cameraType().hasIMU()) {
          io.setIMUMode(IMUMode.INTERNAL_EXTERNAL_ASSIST);
          io.setIMUAssistAlpha(
              limelightConfig.imuAssistAlpha() > 0 ? limelightConfig.imuAssistAlpha() : 0.001);
        } else {
          io.setIMUMode(IMUMode.EXTERNAL);
        }
        io.setThrottle(0);
      }

      if (Timer.getTimestamp() - enabledTimestamp
              >= VisionConstants.REWIND_CAPTURE_INTERVAL.in(org.wpilib.units.Units.Seconds)
          && limelightConfig.enableRewind()) {
        io.triggerRewindCapture(VisionConstants.REWIND_CAPTURE_INTERVAL);
        enabledTimestamp = Timer.getTimestamp();
      }
    }

    if (RobotState.isDisabled()) {
      if (wasEnabled) {
        if (limelightConfig.enableRewind()) {
          io.triggerRewindCapture(
              org.wpilib.units.Units.Seconds.of(Timer.getTimestamp() - enabledTimestamp));
        }
        wasEnabled = false;
        if (limelightConfig.cameraType() != null && limelightConfig.cameraType().hasIMU()) {
          io.setIMUMode(IMUMode.EXTERNAL_SEED_INTERNAL);
        } else {
          io.setIMUMode(IMUMode.EXTERNAL);
        }
        io.setThrottle(GompeiLib.isTuning() ? 0 : 190);
      }
    }

    publishOrientation();
    Logger.recordOutput("Vision/Cameras/" + this.name + "/RealTimeCameraPose", realTimeCameraPose);

    io.updateInputs(inputs);
    Logger.processInputs("Vision/Cameras/" + this.name, inputs);

    allTagPoses.clear();

    // Filters for MT1
    boolean mt1Valid =
        !isStaleFrame(inputs.mt1PoseEstimate.timestampSeconds(), lastMT1TimestampSeconds)
            && isValidTagCount(inputs.mt1PoseEstimate.tagCount())
            && isValidDistance(
                org.wpilib.units.Units.Meters.of(inputs.mt1PoseEstimate.avgTagDist()),
                VisionConstants.MAX_TAG_DISTANCE);

    ResolvedCapture mt1Capture = null;
    if (mt1Valid) {
      mt1Capture =
          resolveCapture(inputs.mt1PoseEstimate.pose(), inputs.mt1PoseEstimate.timestampSeconds());
      if (!isWithinFieldBounds(mt1Capture.robotPose(), VisionConstants.FIELD_BORDER_MARGIN)) {
        mt1Valid = false;
      }
    }

    if (mt1Valid && inputs.mt1PoseEstimate.tagCount() == 1) {
      double ambiguity =
          Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
              .mapToDouble(CameraIO.RawFiducial::ambiguity)
              .findFirst()
              .orElse(1.0);
      if (!isValidAmbiguity(ambiguity)) {
        mt1Valid = false;
      }
    }

    // Filters for MT2
    boolean mt2Valid =
        !isStaleFrame(inputs.mt2PoseEstimate.timestampSeconds(), lastMT2TimestampSeconds)
            && isValidTagCount(inputs.mt2PoseEstimate.tagCount())
            && isValidYawRate(
                org.wpilib.units.Units.RadiansPerSecond.of(chassisSpeedsSupplier.get().omega))
            && isValidDistance(
                org.wpilib.units.Units.Meters.of(inputs.mt2PoseEstimate.avgTagDist()),
                VisionConstants.MAX_TAG_DISTANCE);

    ResolvedCapture mt2Capture = null;
    if (mt2Valid) {
      mt2Capture =
          resolveCapture(inputs.mt2PoseEstimate.pose(), inputs.mt2PoseEstimate.timestampSeconds());
      if (!isWithinFieldBounds(mt2Capture.robotPose(), VisionConstants.FIELD_BORDER_MARGIN)) {
        mt2Valid = false;
      }
    }

    if (mt2Valid) {
      lastMT2TimestampSeconds = inputs.mt2PoseEstimate.timestampSeconds();
      double xyStdDev =
          limelightConfig.megatag2XYStdev()
              * Math.pow(
                  inputs.mt2PoseEstimate.avgTagDist(), VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
              / Math.pow(
                  inputs.mt2PoseEstimate.tagCount(), VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);

      Logger.recordOutput("Vision/Cameras/" + this.name + "/MT2Pose", mt2Capture.robotPose());
      Logger.recordOutput(
          "Vision/Cameras/" + this.name + "/CaptureCameraPose", mt2Capture.cameraPose());
      poseObservationList.add(
          new VisionPoseObservation(
              mt2Capture.robotPose(),
              Arrays.stream(inputs.mt2PoseEstimate.rawFiducials())
                  .map(CameraIO.RawFiducial::id)
                  .collect(Collectors.toSet()),
              inputs.mt2PoseEstimate.timestampSeconds(),
              VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY)));

      // MT1 provides theta only when gate passes
      if (mt1Valid) {
        lastMT1TimestampSeconds = inputs.mt1PoseEstimate.timestampSeconds();
        boolean passesThetaGate =
            inputs.mt1PoseEstimate.tagCount() > 1
                && org.wpilib.units.Units.MetersPerSecond.of(
                        Math.abs(chassisSpeedsSupplier.get().vx))
                    .lte(VisionConstants.THETA_GATE_MAX_VELOCITY)
                && org.wpilib.units.Units.MetersPerSecond.of(
                        Math.abs(chassisSpeedsSupplier.get().vy))
                    .lte(VisionConstants.THETA_GATE_MAX_VELOCITY)
                && org.wpilib.units.Units.RadiansPerSecond.of(
                        Math.abs(chassisSpeedsSupplier.get().omega))
                    .lte(VisionConstants.THETA_GATE_MAX_ANGULAR_VELOCITY)
                && Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
                        .mapToDouble(CameraIO.RawFiducial::ambiguity)
                        .average()
                        .orElse(Double.MAX_VALUE)
                    < VisionConstants.AMBIGUITY_THRESHOLD;

        if (passesThetaGate) {
          double thetaStdev =
              limelightConfig.metatagThetaStdev()
                  * Math.pow(
                      inputs.mt1PoseEstimate.avgTagDist(),
                      VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
                  / Math.pow(
                      inputs.mt1PoseEstimate.tagCount(),
                      VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);

          Logger.recordOutput("Vision/Cameras/" + this.name + "/MT1Pose", mt1Capture.robotPose());
          poseObservationList.add(
              new VisionPoseObservation(
                  mt1Capture.robotPose(),
                  Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
                      .map(CameraIO.RawFiducial::id)
                      .collect(Collectors.toSet()),
                  inputs.mt1PoseEstimate.timestampSeconds(),
                  VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, thetaStdev)));
        }
      }
    } else if (mt1Valid) {
      lastMT1TimestampSeconds = inputs.mt1PoseEstimate.timestampSeconds();
      double xyStdDev =
          limelightConfig.megatagXYStdev()
              * Math.pow(
                  inputs.mt1PoseEstimate.avgTagDist(), VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
              / Math.pow(
                  inputs.mt1PoseEstimate.tagCount(), VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);

      boolean passesThetaGate =
          inputs.mt1PoseEstimate.tagCount() > 1
              && org.wpilib.units.Units.MetersPerSecond.of(Math.abs(chassisSpeedsSupplier.get().vx))
                  .lte(VisionConstants.THETA_GATE_MAX_VELOCITY)
              && org.wpilib.units.Units.MetersPerSecond.of(Math.abs(chassisSpeedsSupplier.get().vy))
                  .lte(VisionConstants.THETA_GATE_MAX_VELOCITY)
              && org.wpilib.units.Units.RadiansPerSecond.of(
                      Math.abs(chassisSpeedsSupplier.get().omega))
                  .lte(VisionConstants.THETA_GATE_MAX_ANGULAR_VELOCITY)
              && Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
                      .mapToDouble(CameraIO.RawFiducial::ambiguity)
                      .average()
                      .orElse(Double.MAX_VALUE)
                  < VisionConstants.AMBIGUITY_THRESHOLD;

      double thetaStdev =
          passesThetaGate
              ? limelightConfig.metatagThetaStdev()
                  * Math.pow(
                      inputs.mt1PoseEstimate.avgTagDist(),
                      VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
                  / Math.pow(
                      inputs.mt1PoseEstimate.tagCount(),
                      VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT)
              : Double.POSITIVE_INFINITY;

      Logger.recordOutput("Vision/Cameras/" + this.name + "/MT1Pose", mt1Capture.robotPose());
      Logger.recordOutput(
          "Vision/Cameras/" + this.name + "/CaptureCameraPose", mt1Capture.cameraPose());
      poseObservationList.add(
          new VisionPoseObservation(
              mt1Capture.robotPose(),
              Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
                  .map(CameraIO.RawFiducial::id)
                  .collect(Collectors.toSet()),
              inputs.mt1PoseEstimate.timestampSeconds(),
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdev)));
    }

    if (inputs.rawFiducials != null && inputs.rawFiducials.length != 0) {
      double fiducialsTimestamp =
          inputs.mt1PoseEstimate.timestampSeconds() > 0
              ? inputs.mt1PoseEstimate.timestampSeconds()
              : inputs.mt2PoseEstimate.timestampSeconds();
      Pose3d fiducialsCamPose =
          mt1Valid
              ? mt1Capture.cameraPose()
              : (mt2Valid ? mt2Capture.cameraPose() : realTimeCameraPose);

      Arrays.stream(inputs.rawFiducials)
          .forEach(
              fiducial ->
                  singleTxTyObservationList.add(
                      new VisionSingleTxTyObservation(
                          fiducial.id(),
                          org.wpilib.units.Units.Degrees.of(fiducial.txnc()),
                          org.wpilib.units.Units.Degrees.of(fiducial.tync()),
                          org.wpilib.units.Units.Meters.of(fiducial.distToCamera()),
                          org.wpilib.units.Units.Seconds.of(fiducialsTimestamp),
                          fiducialsCamPose)));
    }

    super.sendObservers();
  }
}
