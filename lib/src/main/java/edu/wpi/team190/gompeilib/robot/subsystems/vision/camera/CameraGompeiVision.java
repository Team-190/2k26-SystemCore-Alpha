package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.math.GeometryUtil;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants.GompeiVisionConfig;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIOGompeiVision;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.GompeiVisionIOInputsAutoLogged;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.*;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.system.Timer;

@ExtensionMethod(GeometryUtil.class)
public class CameraGompeiVision extends Camera {
  private final GompeiVisionIOInputsAutoLogged inputs;
  private final CameraIOGompeiVision io;

  private final GompeiVisionConfig config;
  private final Supplier<Field> aprilTagFieldLayoutSupplier;
  private final double fieldBorderMarginMeters;
  private final Supplier<Pose2d> currentRobotPoseSupplier;

  @Getter private final String name;

  @Getter private final List<Pose3d> allTagPoses;
  @Getter private Pose2d robotPose;

  private final TimeInterpolatableBuffer<Rotation2d> robotRotationBuffer =
      TimeInterpolatableBuffer.createBuffer(1.5);

  public CameraGompeiVision(
      CameraIOGompeiVision io,
      GompeiVisionConfig config,
      Supplier<Field> aprilTagFieldLayoutSupplier,
      double fieldBorderMarginMeters,
      Supplier<Pose2d> currentRobotPoseSupplier,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionMultiTxTyObservation>>> txtyObservers) {
    super(config.key(), poseObservers, txtyObservers, new ArrayList<>());

    inputs = new GompeiVisionIOInputsAutoLogged();
    this.io = io;

    this.config = config;
    this.aprilTagFieldLayoutSupplier = aprilTagFieldLayoutSupplier;
    this.fieldBorderMarginMeters = fieldBorderMarginMeters;
    this.currentRobotPoseSupplier = currentRobotPoseSupplier;
    setFieldLayoutSupplier(aprilTagFieldLayoutSupplier);

    this.name = this.config.key();

    allTagPoses = new ArrayList<>();
    robotPose = Pose2d.ZERO;

    currentCameraPose = config.robotRelativePose();
  }

  @Override
  @Trace
  public void periodic() {
    poseObservationList.clear();
    multiTxTyObservationList.clear();

    Rotation2d liveRotation = currentRobotPoseSupplier.get().getRotation();
    robotRotationBuffer.addSample(Timer.getTimestamp(), liveRotation);

    io.updateInputs(inputs);
    Logger.processInputs("Vision/Cameras/" + this.name, inputs);

    allTagPoses.clear();
    robotPose = Pose2d.ZERO;

    for (int i = 0; i < inputs.frames.length; i++) {
      double timestamp = inputs.timestamps[i];
      if (timestamp <= 0.0) {
        continue;
      }

      double[] values = inputs.frames[i];

      if (values.length == 0 || values[0] == 0) {
        continue;
      }

      Pose3d cameraPose = null;
      Pose2d robotPose = null;
      Pose3d robotPose3d = null;
      Transform3d cameraToRobot =
          new Transform3d(currentCameraPose.getTranslation(), currentCameraPose.getRotation())
              .inverse();

      switch ((int) values[0]) {
        case 1:
          // One pose
          cameraPose =
              new Pose3d(
                  values[2],
                  values[3],
                  values[4],
                  new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));
          robotPose3d = cameraPose.transformBy(cameraToRobot);
          robotPose = robotPose3d.toPose2d();
          break;

        case 2:
          // Multiple poses
          double error0 = values[1];
          double error1 = values[9];
          Pose3d cameraPose0 =
              new Pose3d(
                  values[2],
                  values[3],
                  values[4],
                  new Rotation3d(new Quaternion(values[5], values[6], values[7], values[8])));

          Pose3d cameraPose1 =
              new Pose3d(
                  values[10],
                  values[11],
                  values[12],
                  new Rotation3d(new Quaternion(values[13], values[14], values[15], values[16])));

          Pose3d robotPose0_3d = cameraPose0.transformBy(cameraToRobot);
          Pose3d robotPose1_3d = cameraPose1.transformBy(cameraToRobot);
          Pose2d robotPose0 = robotPose0_3d.toPose2d();
          Pose2d robotPose1 = robotPose1_3d.toPose2d();

          if (error0 < error1 * VisionConstants.AMBIGUITY_THRESHOLD) {
            robotPose = robotPose0;
            cameraPose = cameraPose0;
            robotPose3d = robotPose0_3d;
          } else if (error1 < error0 * VisionConstants.AMBIGUITY_THRESHOLD) {
            robotPose = robotPose1;
            cameraPose = cameraPose1;
            robotPose3d = robotPose1_3d;
          } else {
            Rotation2d currentRotation =
                robotRotationBuffer.getSample(timestamp).orElse(liveRotation);
            if (Math.abs(currentRotation.minus(robotPose0.getRotation()).getRadians())
                < Math.abs(currentRotation.minus(robotPose1.getRotation()).getRadians())) {
              robotPose = robotPose0;
              cameraPose = cameraPose0;
              robotPose3d = robotPose0_3d;
            } else {
              robotPose = robotPose1;
              cameraPose = cameraPose1;
              robotPose3d = robotPose1_3d;
            }
          }

          break;
        default:
          DriverStationBackend.reportWarning("FAILED TO CAPTURE FRAMES", false);
          continue;
      }

      // Reject null or physically invalid 3D robot pose
      if (cameraPose == null || robotPose3d == null || !isValid3DPose(robotPose3d)) {
        continue;
      }

      // Exit if robot pose is off the field
      if (!isWithinFieldBounds(robotPose, fieldBorderMarginMeters)) {
        continue;
      }

      // Get tag poses and update last detection times
      List<Pose3d> tagPoses = new ArrayList<>();
      for (int j = (values[0] == 1 ? 9 : 17); j < values.length; j += 10) {
        int tagId = (int) values[j];
        Optional<Pose3d> tagPose = aprilTagFieldLayoutSupplier.get().getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Skip observation if no tags resolve
      if (tagPoses.isEmpty()) {
        continue;
      }

      int totalTargets = tagPoses.size();
      Set<Integer> tagIds = new HashSet<>();

      // Calculate average distance to solved field-space cameraPose
      double totalDistance = 0.0;
      for (Pose3d tagPose : tagPoses) {
        totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
      }
      double averageDistance = totalDistance / tagPoses.size();

      // --- Parse tag angle + distance data ---
      int tagEstimationDataEndIndex = (int) values[0] == 1 ? 8 : 16;
      int indexCounter = 0;

      // Step through each 10-value chunk safely
      for (int index = tagEstimationDataEndIndex + 1; index + 9 < values.length; index += 10) {
        int tagId = (int) values[index];
        double[] tx = new double[4];
        double[] ty = new double[4];

        // Read 4 corner pairs using corner index j
        for (int j = 0; j < 4; j++) {
          tx[j] = values[index + 1 + (2 * j)];
          ty[j] = values[index + 1 + (2 * j) + 1];
        }

        double distance = values[index + 9];

        // Store data
        if (indexCounter < totalTargets) {
          indexCounter++;
          multiTxTyObservationList.add(
              new VisionMultiTxTyObservation(tagId, tx, ty, distance, timestamp, cameraPose));
          tagIds.add(tagId);
        } else {
          System.out.println("[WARN] More tag data than expected: indexCounter=" + indexCounter);
        }
      }

      // Optional debug check
      if ((values.length - (tagEstimationDataEndIndex + 1)) % 10 != 0) {
        System.out.println(
            "[WARN] Observation array not multiple of 10! Length="
                + values.length
                + " start="
                + (tagEstimationDataEndIndex + 1));
      }

      double xyStdevCoeff;
      double thetaStdev;

      if (totalTargets > 1) {
        xyStdevCoeff = config.multitagXYStdev();
        thetaStdev =
            config.thetaStdev()
                * Math.pow(averageDistance, VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
                / Math.pow(totalTargets, VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);
      } else {
        xyStdevCoeff = config.singletagXYStdev();
        thetaStdev = Double.POSITIVE_INFINITY;
      }

      // Add observation to list
      double xyStdDev =
          xyStdevCoeff
              * Math.pow(averageDistance, VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
              / Math.pow(totalTargets, VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);

      poseObservationList.add(
          new VisionPoseObservation(
              robotPose, tagIds, timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdev)));
    }

    super.sendObservers();
  }
}
