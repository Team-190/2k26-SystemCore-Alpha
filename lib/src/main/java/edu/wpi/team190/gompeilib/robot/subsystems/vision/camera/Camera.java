package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionMultiTxTyObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionSingleTxTyObservation;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;

public abstract class Camera {
  @Getter private final String name;
  List<Consumer<List<VisionPoseObservation>>> poseObservers;
  List<Consumer<List<VisionMultiTxTyObservation>>> multiTxTyObservers;
  List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers;

  @Getter List<VisionPoseObservation> poseObservationList;
  @Getter List<VisionMultiTxTyObservation> multiTxTyObservationList;
  @Getter List<VisionSingleTxTyObservation> singleTxTyObservationList;

  @Getter protected Pose3d realTimeCameraPose;
  @Getter @Setter protected Supplier<Field> fieldLayoutSupplier;

  public Camera(
      String name,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionMultiTxTyObservation>>> multiTxTyObservers,
      List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
    this.name = name;
    this.poseObservers = new ArrayList<>(poseObservers);
    this.multiTxTyObservers = new ArrayList<>(multiTxTyObservers);
    this.singleTxTyObservers = new ArrayList<>(singleTxTyObservers);

    poseObservationList = new ArrayList<>();
    multiTxTyObservationList = new ArrayList<>();
    singleTxTyObservationList = new ArrayList<>();
  }

  @Trace
  public void periodic() {}

  public void setCameraPose(Pose3d cameraPose) {
    realTimeCameraPose = cameraPose;
  }

  /**
   * @deprecated Use {@link #getRealTimeCameraPose()} instead.
   */
  @Deprecated
  public Pose3d getCurrentCameraPose() {
    return realTimeCameraPose;
  }

  public void sendObservers() {
    List<VisionPoseObservation> immutablePoseObs = List.copyOf(poseObservationList);
    for (Consumer<List<VisionPoseObservation>> observer : poseObservers) {
      observer.accept(immutablePoseObs);
    }
    List<VisionMultiTxTyObservation> immutableMultiTxTyObs = List.copyOf(multiTxTyObservationList);
    for (Consumer<List<VisionMultiTxTyObservation>> observer : multiTxTyObservers) {
      observer.accept(immutableMultiTxTyObs);
    }
    List<VisionSingleTxTyObservation> immutableSingleTxTyObs =
        List.copyOf(singleTxTyObservationList);
    for (Consumer<List<VisionSingleTxTyObservation>> observer : singleTxTyObservers) {
      observer.accept(immutableSingleTxTyObs);
    }
  }

  public boolean isStaleFrame(double timestamp, double lastTimestamp) {
    return timestamp <= 0.0 || timestamp <= lastTimestamp;
  }

  public boolean isValidTagCount(int tagCount) {
    return tagCount > 0;
  }

  public boolean isWithinFieldBounds(Pose2d pose, org.wpilib.units.measure.Distance margin) {
    return isWithinFieldBounds(pose, margin.in(org.wpilib.units.Units.Meters));
  }

  public boolean isWithinFieldBounds(Pose2d pose, double margin) {
    if (pose == null) {
      return false;
    }
    if (fieldLayoutSupplier == null || fieldLayoutSupplier.get() == null) {
      return true;
    }
    Field field = fieldLayoutSupplier.get();
    return pose.getX() >= -margin
        && pose.getX() <= field.getFieldLength() + margin
        && pose.getY() >= -margin
        && pose.getY() <= field.getFieldWidth() + margin;
  }

  public boolean isValid3DPose(Pose3d pose3d) {
    if (pose3d == null) {
      return false;
    }
    double z = pose3d.getZ();
    double roll = Math.abs(pose3d.getRotation().getX());
    double pitch = Math.abs(pose3d.getRotation().getY());
    return z >= -0.8 && z <= 1.0 && roll <= Math.toRadians(35) && pitch <= Math.toRadians(35);
  }

  public boolean isValidAmbiguity(double ambiguity) {
    return ambiguity >= 0 && ambiguity < VisionConstants.AMBIGUITY_THRESHOLD;
  }

  public boolean isValidYawRate(org.wpilib.units.measure.AngularVelocity yawRate) {
    return isValidYawRate(yawRate.in(org.wpilib.units.Units.RadiansPerSecond));
  }

  public boolean isValidYawRate(double omegaRadPerSec) {
    return Math.abs(omegaRadPerSec) <= Math.toRadians(720);
  }

  public boolean isValidDistance(
      org.wpilib.units.measure.Distance distance, org.wpilib.units.measure.Distance maxDistance) {
    return isValidDistance(
        distance.in(org.wpilib.units.Units.Meters), maxDistance.in(org.wpilib.units.Units.Meters));
  }

  public boolean isValidDistance(double distance, double maxDistance) {
    return distance > 0 && distance <= maxDistance;
  }
}
