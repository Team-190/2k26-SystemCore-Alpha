package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import static org.wpilib.units.Units.Degrees;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants.StaticLimelightConfig;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionSingleTxTyObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIOLimelight;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.LimelightIOInputsAutoLogged;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.limelight.LimelightHelpers;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.util.Units;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.PubSubOption;
import org.wpilib.system.Timer;

public class CameraStaticLimelight extends Camera {
  private final LimelightIOInputsAutoLogged inputs;
  private final CameraIOLimelight io;

  private final StaticLimelightConfig config;
  @Getter private final String name;

  private final Supplier<Rotation2d> headingSupplier;
  private final Supplier<ChassisVelocities> chassisSpeedsSupplier;
  private final LongSupplier timestampSupplier;
  private final DoubleArrayPublisher headingPublisher;

  @Getter private final List<Pose3d> allTagPoses;

  private boolean wasEnabled;
  private double enabledTimestamp;

  private double lastMT1TimestampSeconds = -1.0;
  private double lastMT2TimestampSeconds = -1.0;

  public CameraStaticLimelight(
      CameraIOLimelight io,
      StaticLimelightConfig config,
      Supplier<Rotation2d> headingSupplier,
      Supplier<ChassisVelocities> chassisSpeedsSupplier,
      LongSupplier timestampSupplier,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
    super(config.key(), poseObservers, new ArrayList<>(), singleTxTyObservers);

    inputs = new LimelightIOInputsAutoLogged();
    this.io = io;

    this.config = config;
    this.name = "limelight-" + this.config.key();

    this.headingSupplier = headingSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.timestampSupplier = timestampSupplier;
    this.headingPublisher =
        NetworkTableInstance.getDefault()
            .getTable(this.name)
            .getDoubleArrayTopic("robot_orientation_set")
            .publish(PubSubOption.periodic(0.005), PubSubOption.SEND_ALL);

    allTagPoses = new ArrayList<>();

    currentCameraPose =
        new Pose3d(
            config.robotToCameraTransform().getTranslation(),
            config.robotToCameraTransform().getRotation());

    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        currentCameraPose.getX(),
        -currentCameraPose.getY(),
        currentCameraPose.getZ(),
        currentCameraPose.getRotation().getMeasureX().in(Degrees),
        -currentCameraPose.getRotation().getMeasureY().in(Degrees),
        -currentCameraPose.getRotation().getMeasureZ().in(Degrees));

    LimelightHelpers.setRewindEnabled(name, config.enableRewind());

    LimelightHelpers.SetIMUMode(name, 1);
    if (GompeiLib.isTuning()) {
      LimelightHelpers.SetThrottle(name, 0);
    } else {
      LimelightHelpers.SetThrottle(name, 190);
    }

    wasEnabled = false;
    enabledTimestamp = Timer.getTimestamp();
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
        LimelightHelpers.SetIMUMode(name, 0);
        LimelightHelpers.SetThrottle(name, 0);
      }

      if (Timer.getTimestamp() - enabledTimestamp >= 165 && config.enableRewind()) {
        LimelightHelpers.triggerRewindCapture(name, 165);
        enabledTimestamp = Timer.getTimestamp();
      }
    }

    if (RobotState.isDisabled()) {
      if (wasEnabled) {
        if (config.enableRewind()) {
          LimelightHelpers.triggerRewindCapture(name, Timer.getTimestamp() - enabledTimestamp);
        }
        wasEnabled = false;
        LimelightHelpers.SetIMUMode(name, 1);
        if (GompeiLib.isTuning()) {
          LimelightHelpers.SetThrottle(name, 0);
        } else {
          LimelightHelpers.SetThrottle(name, 190);
        }
      }
    }

    headingPublisher.set(
        new double[] {
          headingSupplier.get().getDegrees(),
          Units.radiansToDegrees(chassisSpeedsSupplier.get().omega),
          0.0,
          0.0,
          0.0,
          0.0
        },
        timestampSupplier.getAsLong());
    NetworkTableInstance.getDefault().flush();

    io.updateInputs(inputs);
    Logger.processInputs("Vision/Cameras/" + this.name, inputs);

    allTagPoses.clear();

    // Filters for MT1
    boolean mt1Valid =
        !isStaleFrame(inputs.mt1PoseEstimate.timestampSeconds(), lastMT1TimestampSeconds)
            && isValidTagCount(inputs.mt1PoseEstimate.tagCount())
            && isWithinFieldBounds(inputs.mt1PoseEstimate.pose(), 0.5)
            && isValidDistance(inputs.mt1PoseEstimate.avgTagDist(), 10.0);

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
            && isValidYawRate(chassisSpeedsSupplier.get().omega)
            && isWithinFieldBounds(inputs.mt2PoseEstimate.pose(), 0.5)
            && isValidDistance(inputs.mt2PoseEstimate.avgTagDist(), 10.0);

    if (mt2Valid) {
      lastMT2TimestampSeconds = inputs.mt2PoseEstimate.timestampSeconds();
      double xyStdDev =
          config.megatag2XYStdev()
              * Math.pow(
                  inputs.mt2PoseEstimate.avgTagDist(), VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
              / Math.pow(
                  inputs.mt2PoseEstimate.tagCount(), VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);

      poseObservationList.add(
          new VisionPoseObservation(
              inputs.mt2PoseEstimate.pose(),
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
                && Math.abs(chassisSpeedsSupplier.get().vx) <= 0.15
                && Math.abs(chassisSpeedsSupplier.get().vy) <= 0.15
                && Math.abs(chassisSpeedsSupplier.get().omega) <= 0.05
                && Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
                        .mapToDouble(CameraIO.RawFiducial::ambiguity)
                        .average()
                        .orElse(Double.MAX_VALUE)
                    < VisionConstants.AMBIGUITY_THRESHOLD;

        if (passesThetaGate) {
          double thetaStdev =
              config.metatagThetaStdev()
                  * Math.pow(
                      inputs.mt1PoseEstimate.avgTagDist(),
                      VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
                  / Math.pow(
                      inputs.mt1PoseEstimate.tagCount(),
                      VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);

          poseObservationList.add(
              new VisionPoseObservation(
                  inputs.mt1PoseEstimate.pose(),
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
          config.megatagXYStdev()
              * Math.pow(
                  inputs.mt1PoseEstimate.avgTagDist(), VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
              / Math.pow(
                  inputs.mt1PoseEstimate.tagCount(), VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);

      boolean passesThetaGate =
          inputs.mt1PoseEstimate.tagCount() > 1
              && Math.abs(chassisSpeedsSupplier.get().vx) <= 0.15
              && Math.abs(chassisSpeedsSupplier.get().vy) <= 0.15
              && Math.abs(chassisSpeedsSupplier.get().omega) <= 0.05
              && Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
                      .mapToDouble(CameraIO.RawFiducial::ambiguity)
                      .average()
                      .orElse(Double.MAX_VALUE)
                  < VisionConstants.AMBIGUITY_THRESHOLD;

      double thetaStdev =
          passesThetaGate
              ? config.metatagThetaStdev()
                  * Math.pow(
                      inputs.mt1PoseEstimate.avgTagDist(),
                      VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
                  / Math.pow(
                      inputs.mt1PoseEstimate.tagCount(),
                      VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT)
              : Double.POSITIVE_INFINITY;

      poseObservationList.add(
          new VisionPoseObservation(
              inputs.mt1PoseEstimate.pose(),
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
      Arrays.stream(inputs.rawFiducials)
          .forEach(
              fiducial ->
                  singleTxTyObservationList.add(
                      new VisionSingleTxTyObservation(
                          fiducial.id(),
                          fiducial.txnc(),
                          fiducial.tync(),
                          fiducial.distToCamera(),
                          fiducialsTimestamp,
                          currentCameraPose)));
    }

    super.sendObservers();
  }
}
