package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RadiansPerSecond;

import edu.wpi.team190.gompeilib.math.GeometryUtil;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants.MovingLimelightConfig;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionSingleTxTyObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIO;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.interpolation.TimeInterpolatableBuffer;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Camera implementation for a Limelight mounted on a rotating mechanism (e.g. a turret).
 *
 * <p>Resolves captures in a single latency-compensated step:
 *
 * <ol>
 *   <li>Rewinds to the capture timestamp to find the exact turret angle.
 *   <li>Computes the true 3D camera lens pose on the robot at exposure time.
 *   <li>Recovers the camera lens's true field-space pose ({@code rawReportedRobotPose ∘
 *       robotToBelievedCam}).
 *   <li>Transforms into the true robot chassis frame ({@code cameraInField ∘ (robotToTrueCam)⁻¹}).
 * </ol>
 */
public class CameraMovingLimelight extends CameraLimelight {
  private final MovingLimelightConfig config;
  private final Supplier<Rotation2d> rotationAxisSupplier;
  private final Supplier<AngularVelocity> rotationAxisSpeedSupplier;
  private final Transform2d robotToBelievedCam;
  private final TimeInterpolatableBuffer<Rotation2d> turretAngleBuffer =
      TimeInterpolatableBuffer.createBuffer(1.5);

  public CameraMovingLimelight(
      CameraIO io,
      MovingLimelightConfig config,
      Supplier<Rotation2d> headingSupplier,
      Supplier<Rotation2d> rotationAxisSupplier,
      Supplier<AngularVelocity> rotationAxisSpeedSupplier,
      Supplier<ChassisVelocities> chassisSpeedsSupplier,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
    super(
        io,
        config,
        Pose3d.ZERO
            .transformBy(config.robotToRotationAxisTransform())
            .transformBy(config.rotationAxisToLensTransform()),
        headingSupplier,
        chassisSpeedsSupplier,
        poseObservers,
        singleTxTyObservers);
    this.config = config;
    this.rotationAxisSupplier = rotationAxisSupplier;
    this.rotationAxisSpeedSupplier = rotationAxisSpeedSupplier;
    this.robotToBelievedCam =
        GeometryUtil.toTransform2d(
            Pose3d.ZERO
                .transformBy(config.robotToRotationAxisTransform())
                .transformBy(config.rotationAxisToLensTransform())
                .toPose2d());
  }

  public CameraMovingLimelight(
      CameraIO io,
      MovingLimelightConfig config,
      Supplier<Rotation2d> headingSupplier,
      Supplier<Rotation2d> rotationAxisSupplier,
      Supplier<ChassisVelocities> chassisSpeedsSupplier,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
    this(
        io,
        config,
        headingSupplier,
        rotationAxisSupplier,
        () -> RadiansPerSecond.of(0.0),
        chassisSpeedsSupplier,
        poseObservers,
        singleTxTyObservers);
  }

  @Override
  protected void publishOrientation() {
    double now = Timer.getTimestamp();
    Rotation2d liveTurretAngle = rotationAxisSupplier.get();
    turretAngleBuffer.addSample(now, liveTurretAngle);
    realTimeCameraPose =
        Pose3d.ZERO
            .transformBy(config.robotToRotationAxisTransform())
            .transformBy(new Transform3d(new Translation3d(), new Rotation3d(liveTurretAngle)))
            .transformBy(config.rotationAxisToLensTransform());

    AngularVelocity turretSpeed = rotationAxisSpeedSupplier.get();
    AngularVelocity chassisOmega = RadiansPerSecond.of(chassisSpeedsSupplier.get().omega);
    AngularVelocity totalOmega = chassisOmega.plus(turretSpeed);
    Angle totalYaw = Degrees.of(headingSupplier.get().getDegrees() + liveTurretAngle.getDegrees());

    io.setRobotOrientation(totalYaw, totalOmega);
  }

  @Override
  protected ResolvedCapture resolveCapture(Pose2d rawReportedRobotPose, double timestampSeconds) {
    Rotation2d turretAngleAtCapture =
        turretAngleBuffer.getSample(timestampSeconds).orElseGet(rotationAxisSupplier);
    Pose3d cameraPoseAtCapture =
        Pose3d.ZERO
            .transformBy(config.robotToRotationAxisTransform())
            .transformBy(new Transform3d(new Translation3d(), new Rotation3d(turretAngleAtCapture)))
            .transformBy(config.rotationAxisToLensTransform());

    // 1. Recover camera lens in field space: rawReportedRobotPose ∘ robotToBelievedCam
    // 2. Project back to true robot chassis center: (camera on field) ∘ (robotToTrueCam)⁻¹
    Transform2d robotToTrueCam = GeometryUtil.toTransform2d(cameraPoseAtCapture.toPose2d());
    Pose2d trueRobotPose =
        rawReportedRobotPose.transformBy(robotToBelievedCam).transformBy(robotToTrueCam.inverse());

    return new ResolvedCapture(trueRobotPose, cameraPoseAtCapture);
  }
}
