package edu.wpi.team190.gompeilib.robot.subsystems.vision;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.robot.subsystems.vision.camera.CameraType;
import lombok.Builder;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.numbers.N5;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;

public class VisionConstants {
  public static final double AMBIGUITY_THRESHOLD = 0.4;
  public static final double XY_STDEV_DISTANCE_EXPONENT = 1.2;
  public static final double XY_STDEV_TAG_COUNT_EXPONENT = 2.0;

  public static final Distance FIELD_BORDER_MARGIN = Meters.of(0.5);
  public static final Distance MAX_TAG_DISTANCE = Meters.of(10.0);
  public static final AngularVelocity MAX_MT2_YAW_RATE = DegreesPerSecond.of(720.0);
  public static final Time REWIND_CAPTURE_INTERVAL = Seconds.of(165);
  public static final LinearVelocity THETA_GATE_MAX_VELOCITY = MetersPerSecond.of(0.15);
  public static final AngularVelocity THETA_GATE_MAX_ANGULAR_VELOCITY = RadiansPerSecond.of(0.05);

  public interface LimelightConfig {
    String key();

    CameraType cameraType();

    Angle horizontalFOV();

    Angle verticalFOV();

    double megatagXYStdev();

    double metatagThetaStdev();

    double megatag2XYStdev();

    boolean enableRewind();

    double imuAssistAlpha();
  }

  @Builder
  public record StaticLimelightConfig(
      String key,
      CameraType cameraType,
      Angle horizontalFOV,
      Angle verticalFOV,
      double megatagXYStdev,
      double metatagThetaStdev,
      double megatag2XYStdev,
      Transform3d robotToCameraTransform,
      boolean enableRewind,
      double imuAssistAlpha)
      implements LimelightConfig {}

  @Builder
  public record MovingLimelightConfig(
      String key,
      CameraType cameraType,
      Angle horizontalFOV,
      Angle verticalFOV,
      double megatagXYStdev,
      double metatagThetaStdev,
      double megatag2XYStdev,
      Transform3d robotToRotationAxisTransform,
      Transform3d rotationAxisToLensTransform,
      boolean enableRewind,
      double imuAssistAlpha)
      implements LimelightConfig {}

  @Builder
  public record GompeiVisionConfig(
      String key,
      String hardwareID,
      CameraType cameraType,
      Time exposure,
      double gain,
      int width,
      int height,
      Matrix<N3, N3> cameraMatrix,
      Matrix<N5, N1> distortionCoefficients,
      Angle horizontalFOV,
      Angle verticalFOV,
      double singletagXYStdev,
      double thetaStdev,
      double multitagXYStdev,
      Pose3d robotRelativePose) {}
}
