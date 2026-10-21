package first.robot.v3_Horse_CV2_TR;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.phoenix.CANBuses;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveModuleConfig;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.phoenix.SwerveModuleConstantsConversions;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants.StaticLimelightConfig;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.camera.CameraType;
import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearVelocityUnit;

public class V3_Horse_CV2_TRConstants {
  public static final CANBusId DRIVE_BUS =
      CANBuses.toCANBusId(V3_Horse_CV2_TRTunerConstants.kCANBus);

  private static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor TURN_MOTOR = DCMotor.getKrakenX44Foc(1);

  public static final DriveConfig DRIVE_CONFIG =
      new DriveConfig(
          DRIVE_BUS,
          V3_Horse_CV2_TRTunerConstants.DrivetrainConstants.Pigeon2Id,
          V3_Horse_CV2_TRTunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
          V3_Horse_CV2_TRTunerConstants.kWheelRadius.in(Meters),
          DRIVE_MOTOR,
          TURN_MOTOR,
          module(V3_Horse_CV2_TRTunerConstants.FrontLeft),
          module(V3_Horse_CV2_TRTunerConstants.FrontRight),
          module(V3_Horse_CV2_TRTunerConstants.BackLeft),
          module(V3_Horse_CV2_TRTunerConstants.BackRight),
          SwerveModuleConstantsConversions.toOutputType(
              V3_Horse_CV2_TRTunerConstants.kDriveClosedLoopOutput),
          SwerveModuleConstantsConversions.toOutputType(
              V3_Horse_CV2_TRTunerConstants.kSteerClosedLoopOutput),
          Units.inchesToMeters(34.25), // width
          Units.inchesToMeters(32.25), // length
          0.0,
          0.0,
          0.0,
          0.0,
          2.0);

  // Tuner X gains are per wheel (or module) rotation, in the output its closed-loop type uses.
  public static final Gains<?, AngularVelocityUnit, AngularVelocityUnit> DRIVE_GAINS =
      SwerveModuleConstantsConversions.toVelocityGains(
          "Drive/Teleoperated/Drive",
          V3_Horse_CV2_TRTunerConstants.driveGains,
          V3_Horse_CV2_TRTunerConstants.kDriveClosedLoopOutput);

  public static final Gains<?, AngleUnit, AngularVelocityUnit> TURN_GAINS =
      SwerveModuleConstantsConversions.toPositionGains(
          "Drive/Teleoperated/Turn",
          V3_Horse_CV2_TRTunerConstants.steerGains,
          V3_Horse_CV2_TRTunerConstants.kSteerClosedLoopOutput);

  public static final Gains<LinearVelocityUnit, DistanceUnit, LinearVelocityUnit>
      TRANSLATION_AUTO_GAINS = // TODO: Use Real Value
      Gains.linearPosition(MetersPerSecond)
              .withPrefix("Drive/Auto/Translation")
              .withKP(MetersPerSecond.per(Meters).ofNative(5.0))
              .withKD(MetersPerSecond.per(MetersPerSecond).ofNative(0.0))
              .build();

  public static final Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit>
      ROTATION_AUTO_GAINS = // TODO: Use Real Value
      Gains.builder(RadiansPerSecond, Radians, RadiansPerSecond)
              .withPrefix("Drive/Auto/Rotation")
              .withKP(RadiansPerSecond.per(Radians).ofNative(5.0))
              .withKD(RadiansPerSecond.per(RadiansPerSecond).ofNative(0.0))
              .build();

  public static final Gains<LinearVelocityUnit, DistanceUnit, LinearVelocityUnit>
      AUTO_ALIGN_X_GAINS = // TODO: Use Real Value
      Gains.linearPosition(MetersPerSecond)
              .withPrefix("Drive/Auto Align/X")
              .withKP(MetersPerSecond.per(Meters).ofNative(3.0))
              .withKD(MetersPerSecond.per(MetersPerSecond).ofNative(0.15))
              .build();

  public static final LinearConstraints AUTO_ALIGN_X_CONSTRAINTS = // TODO: Use Real Value
      LinearConstraints.builder()
          .withPrefix("Drive/Auto Align/X")
          .withMaxVelocity(MetersPerSecond.of(2.5))
          .withMaxAcceleration(MetersPerSecondPerSecond.of(4.0))
          .withGoalTolerance(Meters.of(0.03))
          .build();

  public static final Gains<LinearVelocityUnit, DistanceUnit, LinearVelocityUnit>
      AUTO_ALIGN_Y_GAINS = // TODO: Use Real Value
      Gains.linearPosition(MetersPerSecond)
              .withPrefix("Drive/Auto Align/Y")
              .withKP(MetersPerSecond.per(Meters).ofNative(3.0))
              .withKD(MetersPerSecond.per(MetersPerSecond).ofNative(0.15))
              .build();

  public static final LinearConstraints AUTO_ALIGN_Y_CONSTRAINTS = // TODO: Use Real Value
      LinearConstraints.builder()
          .withPrefix("Drive/Auto Align/Y")
          .withMaxVelocity(MetersPerSecond.of(2.5))
          .withMaxAcceleration(MetersPerSecondPerSecond.of(4.0))
          .withGoalTolerance(Meters.of(0.05))
          .build();

  public static final Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit>
      AUTO_ALIGN_THETA_GAINS = // TODO: Use Real Value
      Gains.builder(RadiansPerSecond, Radians, RadiansPerSecond)
              .withPrefix("Drive/Auto Align/Theta")
              .withKP(RadiansPerSecond.per(Radians).ofNative(6.5))
              .withKD(RadiansPerSecond.per(RadiansPerSecond).ofNative(0.2))
              .build();

  public static final AngularPositionConstraints
      AUTO_ALIGN_THETA_CONSTRAINTS = // TODO: Use Real Value
      AngularPositionConstraints.builder()
              .withPrefix("Drive/Auto Align/Theta")
              .withMaxVelocity(RadiansPerSecond.of(Math.PI))
              .withMaxAcceleration(RadiansPerSecondPerSecond.of(4 * Math.PI))
              .withGoalTolerance(Degrees.of(0.5))
              .build();

  public static final SwerveDriveConstants.AutoAlignConstants
      AUTO_ALIGN_CONSTANTS = // TODO: Use Real Value
      SwerveDriveConstants.AutoAlignConstants.builder()
              .withXGains(AUTO_ALIGN_X_GAINS)
              .withXConstraints(AUTO_ALIGN_X_CONSTRAINTS)
              .withYGains(AUTO_ALIGN_Y_GAINS)
              .withYConstraints(AUTO_ALIGN_Y_CONSTRAINTS)
              .withRotationGains(AUTO_ALIGN_THETA_GAINS)
              .withRotationConstraints(AUTO_ALIGN_THETA_CONSTRAINTS)
              .withLinearThreshold(
                  GompeiTunables.value("Drive/Auto Align/Position Threshold", Inches.of(0.25)))
              .withAngularThreshold(
                  GompeiTunables.value("Drive/Auto Align/Angular Threshold", Radians.of(0.25)))
              .build();

  // The drive closes its loop in torque current, so it's characterized in amps. Every leg drives
  // the
  // robot straight until it times out, so the timeouts are short: give it room.
  public static final CharacterizationConfig<CurrentUnit> DRIVE_CHARACTERIZATION =
      CharacterizationConfig.current()
          .withPrefix("Drive/Characterization")
          .withRampRate(CustomUnits.ampsPerSecond.ofNative(4))
          .withStepOutput(Amps.of(20))
          .withQuasistaticTimeout(Seconds.of(4))
          .withDynamicTimeout(Seconds.of(1))
          .withSettleTime(Seconds.of(2))
          .build();

  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double DRIVER_DEADBAND = 0.1;
  public static final double OPERATOR_DEADBAND = 0.1;

  /** Converts one module's Tuner X constants, sampling odometry at {@link #ODOMETRY_FREQUENCY}. */
  private static SwerveModuleConfig module(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    return SwerveModuleConstantsConversions.toModuleConfig(
        constants,
        DRIVE_BUS,
        Hertz.of(ODOMETRY_FREQUENCY),
        Seconds.of(0.02),
        DRIVE_MOTOR,
        TURN_MOTOR);
  }

  public static final SwerveDriveConstants DRIVE_CONSTANTS = // TODO: Use Real Value
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

  public static final StaticLimelightConfig LIMELIGHT_SHOOTER_CONFIG = // TODO: Use Real Value
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
                  -0.321,
                  0,
                  0.391,
                  new Rotation3d(
                      Units.degreesToRadians(0),
                      Units.degreesToRadians(20),
                      Units.degreesToRadians(180))))
          .enableRewind(true)
          .build();

  public static final StaticLimelightConfig LIMELIGHT_CLIMBER_CONFIG = // TODO: Use Real Value
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

  public static final StaticLimelightConfig frontcamera = // TODO: Use Real Value
      StaticLimelightConfig.builder()
          .key("front")
          .cameraType(CameraType.LIMELIGHT_4)
          .horizontalFOV(CameraType.LIMELIGHT_4.horizontalFOV)
          .verticalFOV(CameraType.LIMELIGHT_4.verticalFOV)
          .megatagXYStdev(CameraType.LIMELIGHT_4.secondaryXYStandardDeviationCoefficient)
          .metatagThetaStdev(CameraType.LIMELIGHT_4.primaryThetaStandardDeviationCoefficient)
          .megatag2XYStdev(CameraType.LIMELIGHT_4.primaryXYStandardDeviationCoefficient)
          .robotToCameraTransform(new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)))
          .enableRewind(true)
          .build();

  public static final double AUTO_CORRECTION_THRESHOLD_METERS = 0.2;
}
