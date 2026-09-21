package first.robot.v3_Horse_CV2_TR;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.StaticLimelightConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraType;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;

public class V3_Horse_CV2_TRConstants {

  public static final DriveConfig DRIVE_CONFIG =
      new DriveConfig(
          V3_Horse_CV2_TRTunerConstants.kCANBus,
          V3_Horse_CV2_TRTunerConstants.DrivetrainConstants.Pigeon2Id,
          V3_Horse_CV2_TRTunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          V3_Horse_CV2_TRTunerConstants.kWheelRadius.in(Meters),
          DCMotor.getKrakenX60Foc(1),
          DCMotor.getKrakenX44Foc(1),
          V3_Horse_CV2_TRTunerConstants.FrontLeft,
          V3_Horse_CV2_TRTunerConstants.FrontRight,
          V3_Horse_CV2_TRTunerConstants.BackLeft,
          V3_Horse_CV2_TRTunerConstants.BackRight,
          V3_Horse_CV2_TRTunerConstants.kDriveClosedLoopOutput,
          V3_Horse_CV2_TRTunerConstants.kSteerClosedLoopOutput,
          Units.inchesToMeters(37.5), //TODO: Use Real Value
          Units.inchesToMeters(28.5), //TODO: Use Real Value
          0.0,
          0.0,
          0.0,
          0.0,
          2.0);

  public static final Gains DRIVE_GAINS =
      Gains.builder()
          .withKP(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kp", V3_Horse_CV2_TRTunerConstants.driveGains.kP))
          .withKD(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kd", V3_Horse_CV2_TRTunerConstants.driveGains.kD))
          .withKS(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Ks", V3_Horse_CV2_TRTunerConstants.driveGains.kS))
          .withKV(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Drive Kv", V3_Horse_CV2_TRTunerConstants.driveGains.kV))
          .build();

  public static final Gains TURN_GAINS =
      Gains.builder()
          .withKP(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kp", V3_Horse_CV2_TRTunerConstants.steerGains.kP))
          .withKD(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kd", V3_Horse_CV2_TRTunerConstants.steerGains.kD))
          .withKS(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Ks", V3_Horse_CV2_TRTunerConstants.steerGains.kS))
          .withKV(
              new LoggedTunableNumber(
                  "Drive/Teleoperated/Turn Kv", V3_Horse_CV2_TRTunerConstants.steerGains.kV))
          .build();

  public static final Gains TRANSLATION_AUTO_GAINS = //TODO: Use Real Value
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto/Translation Kp", 5.0))
          .withKD(new LoggedTunableNumber("Drive/Auto/Translation Kd", 0.0))
          .build();

  public static final Gains ROTATION_AUTO_GAINS = //TODO: Use Real Value
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto/Rotation Kp", 5.0))
          .withKD(new LoggedTunableNumber("Drive/Auto/Rotation Kd", 0.0))
          .build();

  public static final Gains AUTO_ALIGN_X_GAINS = //TODO: Use Real Value
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/X/Kp", 3.0))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/X/Kd", 0.15))
          .build();

  public static final LinearConstraints AUTO_ALIGN_X_CONSTRAINTS = //TODO: Use Real Value
      LinearConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/X/Max Velocity", MetersPerSecond.of(2.5)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/X/Max Acceleration", MetersPerSecondPerSecond.of(0.0)))
          .withGoalTolerance(
              new LoggedTunableMeasure<>("Drive/Auto Align/X/Goal Tolerance", Meters.of(0.03)))
          .build();

  public static final Gains AUTO_ALIGN_Y_GAINS = //TODO: Use Real Value
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/Y/Kp", 3.0))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/Y/Kd", 0.15))
          .build();

  public static final LinearConstraints AUTO_ALIGN_Y_CONSTRAINTS = //TODO: Use Real Value
      LinearConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Y/Max Velocity", MetersPerSecond.of(2.5)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Y/Max Acceleration", MetersPerSecondPerSecond.of(0.0)))
          .withGoalTolerance(
              new LoggedTunableMeasure<>("Drive/Auto Align/Y/Goal Tolerance", Meters.of(0.05)))
          .build();

  public static final Gains AUTO_ALIGN_THETA_GAINS = //TODO: Use Real Value
      Gains.builder()
          .withKP(new LoggedTunableNumber("Drive/Auto Align/Theta/Kp", 6.5))
          .withKD(new LoggedTunableNumber("Drive/Auto Align/Theta/Kd", 0.2))
          .build();

  public static final AngularPositionConstraints AUTO_ALIGN_THETA_CONSTRAINTS = //TODO: Use Real Value
      AngularPositionConstraints.builder()
          .withMaxVelocity(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Theta/Max Velocity", RadiansPerSecond.of(Math.PI)))
          .withMaxAcceleration(
              new LoggedTunableMeasure<>(
                  "Drive/Auto Align/Theta/Max Acceleration", RadiansPerSecondPerSecond.of(0.0)))
          .withGoalTolerance(
              new LoggedTunableMeasure<>("Drive/Auto Align/Theta/Goal Tolerance", Degrees.of(0.5)))
          .build();

  public static final SwerveDriveConstants.AutoAlignConstants AUTO_ALIGN_CONSTANTS = //TODO: Use Real Value
      SwerveDriveConstants.AutoAlignConstants.builder()
          .withXGains(AUTO_ALIGN_X_GAINS)
          .withXConstraints(AUTO_ALIGN_X_CONSTRAINTS)
          .withYGains(AUTO_ALIGN_Y_GAINS)
          .withYConstraints(AUTO_ALIGN_Y_CONSTRAINTS)
          .withRotationGains(AUTO_ALIGN_THETA_GAINS)
          .withRotationConstraints(AUTO_ALIGN_THETA_CONSTRAINTS)
          .withLinearThreshold(
              new LoggedTunableMeasure<>("Drive/Auto Align/Position Threshold", Inches.of(0.25)))
          .withAngularThreshold(
              new LoggedTunableMeasure<>("Drive/Auto Align/Angular Threshold", Radians.of(0.25)))
          .build();

  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double DRIVER_DEADBAND = 0.1;
  public static final double OPERATOR_DEADBAND = 0.1;

  public static final SwerveDriveConstants DRIVE_CONSTANTS = //TODO: Use Real Value
      SwerveDriveConstants.builder()
          .withDriveConfig(DRIVE_CONFIG)
          .withDriveGains(DRIVE_GAINS)
          .withTurnGains(TURN_GAINS)
          .withAutoTranslationGains(TRANSLATION_AUTO_GAINS)
          .withAutoRotationGains(ROTATION_AUTO_GAINS)
          .withAutoAlignConstants(AUTO_ALIGN_CONSTANTS)
          .withOdometryFrequency(ODOMETRY_FREQUENCY)
          .withDriverDeadband(DRIVER_DEADBAND)
          .withOperatorDeadband(OPERATOR_DEADBAND)
          .build();

  public static final StaticLimelightConfig LIMELIGHT_SHOOTER_CONFIG = //TODO: Use Real Value
      StaticLimelightConfig.builder()
          .key("shooter")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.primaryThetaStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  -0.055,
                  0.054,
                  0.538,
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(90 - 62.000),
                      Units.degreesToRadians(-90.000))))
          .enableRewind(true)
          .build();

  public static final StaticLimelightConfig LIMELIGHT_CLIMBER_CONFIG = //TODO: Use Real Value
      StaticLimelightConfig.builder()
          .key("climber")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.primaryThetaStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(
              new Transform3d(
                  -0.060142,
                  0.398769,
                  0.305,
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(90.409532))))
          .enableRewind(true)
          .build();

  public static final double AUTO_CORRECTION_THRESHOLD_METERS = 0.2;
}
