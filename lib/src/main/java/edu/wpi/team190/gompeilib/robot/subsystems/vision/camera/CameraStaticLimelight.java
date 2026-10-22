package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RadiansPerSecond;

import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants.StaticLimelightConfig;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionSingleTxTyObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIO;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class CameraStaticLimelight extends CameraLimelight {

  public CameraStaticLimelight(
      CameraIO io,
      StaticLimelightConfig config,
      Supplier<Rotation2d> headingSupplier,
      Supplier<ChassisVelocities> chassisSpeedsSupplier,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
    super(
        io,
        config,
        new Pose3d(
            config.robotToCameraTransform().getTranslation(),
            config.robotToCameraTransform().getRotation()),
        headingSupplier,
        chassisSpeedsSupplier,
        poseObservers,
        singleTxTyObservers);
  }

  @Override
  protected void publishOrientation() {
    Angle yaw = Degrees.of(headingSupplier.get().getDegrees());
    AngularVelocity omega = RadiansPerSecond.of(chassisSpeedsSupplier.get().omega);
    io.setSharedRobotOrientation(yaw, omega);
  }
}
