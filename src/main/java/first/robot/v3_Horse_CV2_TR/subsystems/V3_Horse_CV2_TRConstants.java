package first.robot.v3_Horse_CV2_TR.subsystems;

import static org.wpilib.math.util.Units.*;
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
import edu.wpi.team190.gompeilib.core.utility.tunable.GompeiTunables;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import org.wpilib.math.system.DCMotor;

public class V3_Horse_CV2_TRConstants {

  static {
    System.out.println("A");
  }

  public static final DriveConfig DRIVE_CONFIG =
      DriveConfig.builder()
          .withCanBus(V3_Horse_CV2_TRTunerConstants.kCANBus)
          .withPigeon2Id(V3_Horse_CV2_TRTunerConstants.DrivetrainConstants.Pigeon2Id)
          .withMaxLinearVelocityMetersPerSecond(
              V3_Horse_CV2_TRTunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
          .withWheelRadiusMeters(V3_Horse_CV2_TRTunerConstants.kWheelRadius.in(Meters))
          .withDriveModel(DCMotor.getKrakenX60Foc(1))
          .withTurnModel(DCMotor.getKrakenX44Foc(1))
          .withFrontLeft(V3_Horse_CV2_TRTunerConstants.FrontLeft)
          .withFrontRight(V3_Horse_CV2_TRTunerConstants.FrontRight)
          .withBackLeft(V3_Horse_CV2_TRTunerConstants.BackLeft)
          .withBackRight(V3_Horse_CV2_TRTunerConstants.BackRight)
          .withDriveClosedLoopOutputType(V3_Horse_CV2_TRTunerConstants.kDriveClosedLoopOutput)
          .withSteerClosedLoopOutputType(V3_Horse_CV2_TRTunerConstants.kSteerClosedLoopOutput)
          .withBumperWidth(Inches.of(37.5).in(Meters))
          .withBumperLength(Inches.of(28.5).in(Meters))
          .withTrackWidth(Inches.of(0).in(Meters))
          .withRobotMOI(7.897)
          .withModuleCurrentLimit(60.0)
          .withRobotMassKilograms(67.000)
          .withWheelCOF(2.0)
          .build();

  public static final Gains DRIVE_GAINS =
      Gains.builder()
          .withPrefix("Drive/Teleoperated/Drive")
          .withKP(V3_Horse_CV2_TRTunerConstants.driveGains.kP)
          .withKD(V3_Horse_CV2_TRTunerConstants.driveGains.kD)
          .withKS(V3_Horse_CV2_TRTunerConstants.driveGains.kS)
          .withKV(V3_Horse_CV2_TRTunerConstants.driveGains.kV)
          .build();

  public static final Gains TURN_GAINS =
      Gains.builder()
          .withPrefix("Drive/Teleoperated/Turn")
          .withKP(V3_Horse_CV2_TRTunerConstants.steerGains.kP)
          .withKD(V3_Horse_CV2_TRTunerConstants.steerGains.kD)
          .withKS(V3_Horse_CV2_TRTunerConstants.steerGains.kS)
          .withKV(V3_Horse_CV2_TRTunerConstants.steerGains.kV)
          .build();

  public static final Gains TRANSLATION_AUTO_GAINS =
      Gains.builder().withPrefix("Drive/Auto/Translation").withKP(5.0).withKD(0.0).build();

  public static final Gains ROTATION_AUTO_GAINS =
      Gains.builder().withPrefix("Drive/Auto/Rotation").withKP(5.0).withKD(0.0).build();

  public static final Gains AUTO_ALIGN_X_GAINS =
      Gains.builder().withPrefix("Drive/Auto Align/X").withKP(3.0).withKD(0.15).build();

  public static final LinearConstraints AUTO_ALIGN_X_CONSTRAINTS =
      LinearConstraints.builder()
          .withPrefix("Drive/Auto Align/X")
          .withMaxVelocity(MetersPerSecond.of(2.5))
          .withMaxAcceleration(MetersPerSecondPerSecond.of(0.0))
          .withGoalTolerance(Meters.of(0.03))
          .build();

  public static final Gains AUTO_ALIGN_Y_GAINS =
      Gains.builder().withPrefix("Drive/Auto Align/Y").withKP(3.0).withKD(0.15).build();

  public static final LinearConstraints AUTO_ALIGN_Y_CONSTRAINTS =
      LinearConstraints.builder()
          .withPrefix("Drive/Auto Align/Y")
          .withMaxVelocity(MetersPerSecond.of(2.5))
          .withMaxAcceleration(MetersPerSecondPerSecond.of(0.0))
          .withGoalTolerance(Meters.of(0.05))
          .build();

  public static final Gains AUTO_ALIGN_THETA_GAINS =
      Gains.builder()
          .withPrefix("Drive/Auto Align/Theta")
          .withKP(2.0 * Math.PI)
          .withKD(0.05)
          .build();

  public static final AngularPositionConstraints AUTO_ALIGN_THETA_CONSTRAINTS =
      AngularPositionConstraints.builder()
          .withPrefix("Drive/Auto Align/Theta")
          .withMaxVelocity(RadiansPerSecond.of(Math.PI))
          .withMaxAcceleration(RadiansPerSecondPerSecond.of(0.0))
          .withGoalTolerance(Degrees.of(0.5))
          .build();

  public static final SwerveDriveConstants.AutoAlignConstants AUTO_ALIGN_CONSTANTS =
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

  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double DRIVER_DEADBAND = 0.1;
  public static final double OPERATOR_DEADBAND = 0.1;

  public static final SwerveDriveConstants DRIVE_CONSTANTS =
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
}
