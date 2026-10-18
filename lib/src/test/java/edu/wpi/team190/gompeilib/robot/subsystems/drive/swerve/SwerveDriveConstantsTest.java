package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.TestGains;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.gyro.config.GyroConfig;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.system.DCMotor;
import org.wpilib.tunable.Tunable;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearVelocityUnit;

public class SwerveDriveConstantsTest extends GompeiLibTestBase {

  /** Chassis translation gains, in meters per second per meter of error. */
  public static Gains<LinearVelocityUnit, DistanceUnit, LinearVelocityUnit> translationGains(
      double kP, double kD) {
    return Gains.linearPosition(MetersPerSecond)
        .withKP(MetersPerSecond.per(Meters).ofNative(kP))
        .withKD(MetersPerSecond.per(MetersPerSecond).ofNative(kD))
        .build();
  }

  /** Chassis rotation gains, in radians per second per radian of error. */
  public static Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit> rotationGains(
      double kP, double kD) {
    return Gains.builder(RadiansPerSecond, Radians, RadiansPerSecond)
        .withKP(RadiansPerSecond.per(Radians).ofNative(kP))
        .withKD(RadiansPerSecond.per(RadiansPerSecond).ofNative(kD))
        .build();
  }

  public static SwerveModuleConfig createModuleConfig(
      int driveId, int steerId, int encoderId, double x, double y) {
    return createModuleConfig(driveId, steerId, encoderId, x, y, OutputType.VOLTAGE);
  }

  /** A module whose drive slot holds a 0.2 kS in the given output. */
  public static SwerveModuleConfig createModuleConfig(
      int driveId, int steerId, int encoderId, double x, double y, OutputType driveOutput) {
    CANBusId bus = CANBusId.named("rio");
    SlotConfig driveSlot =
        driveOutput == OutputType.CURRENT
            ? SlotConfig.velocity(Gains.angularVelocity(Amps).withKS(Amps.of(0.2)).build())
            : SlotConfig.velocity(TestGains.velocity(0, 0, 0, 0.2, 0, 0, 0));
    CurrentLimits limits = new CurrentLimits(Amps.of(60.0), Amps.of(80.0));
    return SwerveModuleConfig.builder()
        .withDriveMotor(
            MotorConfig.builder()
                .withCanId(driveId)
                .withBus(bus)
                .withCurrentLimits(limits)
                .withGearRatio(6.0)
                .withSlot(GainSlot.ZERO, driveSlot)
                .build())
        .withTurnMotor(
            MotorConfig.builder()
                .withCanId(steerId)
                .withBus(bus)
                .withCurrentLimits(limits)
                .withGearRatio(10.0)
                .build())
        .withEncoder(AbsoluteEncoderConfig.builder().withCanId(encoderId).withBus(bus).build())
        .withLocation(new Translation2d(x, y))
        .withDriveInertia(KilogramSquareMeters.of(0.01))
        .withTurnInertia(KilogramSquareMeters.of(0.01))
        .build();
  }

  @Test
  public void testConstantsAndBuilders() {
    CANBusId canBus = CANBusId.named("rio");
    DCMotor driveModel = DCMotor.getKrakenX60(1);
    DCMotor turnModel = DCMotor.getKrakenX44(1);

    var fl = createModuleConfig(1, 2, 3, 0.5, 0.5);
    var fr = createModuleConfig(4, 5, 6, 0.5, -0.5);
    var bl = createModuleConfig(7, 8, 9, -0.5, 0.5);
    var br = createModuleConfig(10, 11, 12, -0.5, -0.5);

    SwerveDriveConstants.DriveConfig driveConfig =
        SwerveDriveConstants.DriveConfig.builder()
            .withCanBus(canBus)
            .withPigeon2Id(13)
            .withMaxLinearVelocityMetersPerSecond(5.0)
            .withWheelRadiusMeters(0.05)
            .withDriveModel(driveModel)
            .withTurnModel(turnModel)
            .withFrontLeft(fl)
            .withFrontRight(fr)
            .withBackLeft(bl)
            .withBackRight(br)
            .withDriveClosedLoopOutputType(OutputType.VOLTAGE)
            .withSteerClosedLoopOutputType(OutputType.VOLTAGE)
            .withBumperWidth(0.8)
            .withBumperLength(0.8)
            .withRobotMassKilograms(45.0)
            .withTrackWidth(0.6)
            .withRobotMOI(2.5)
            .withModuleCurrentLimit(40.0)
            .withWheelCOF(1.2)
            .build();

    var driveGains = TestGains.velocity(1.0, 0, 0.1, 0, 0, 0, 0);
    var turnGains = TestGains.position(2.0, 0, 0.2, 0, 0, 0, 0);
    var translationGains = translationGains(1.0, 0.1);
    var rotationGains = rotationGains(2.0, 0.2);

    SwerveDriveConstants.AutoAlignConstants autoAlignConstants =
        SwerveDriveConstants.AutoAlignConstants.builder()
            .withXGains(translationGains)
            .withXConstraints(
                LinearConstraints.builder()
                    .withMaxVelocity(MetersPerSecond.of(1.0))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Meters.of(0.05))
                    .build())
            .withYGains(translationGains)
            .withYConstraints(
                LinearConstraints.builder()
                    .withMaxVelocity(MetersPerSecond.of(1.0))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Meters.of(0.05))
                    .build())
            .withRotationGains(rotationGains)
            .withRotationConstraints(
                AngularPositionConstraints.builder()
                    .withMaxVelocity(RadiansPerSecond.of(1.0))
                    .withMaxAcceleration(RadiansPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Radians.of(0.05))
                    .build())
            .withLinearThreshold(Tunable.create(Meters.of(0.01)))
            .withAngularThreshold(Tunable.create(Radians.of(0.01)))
            .build();

    SwerveDriveConstants constants =
        SwerveDriveConstants.builder()
            .withDriveConfig(driveConfig)
            .withDriveGains(driveGains)
            .withTurnGains(turnGains)
            .withAutoTranslationGains(translationGains)
            .withAutoRotationGains(rotationGains)
            .withAutoAlignConstants(autoAlignConstants)
            .withOdometryFrequency(250.0)
            .withDriverDeadband(0.1)
            .withOperatorDeadband(0.1)
            .build();

    assertNotNull(constants);
    assertEquals(250.0, constants.odometryFrequency);

    GyroConfig gyro = constants.gyroConfig();
    assertEquals(constants.driveConfig.pigeon2Id(), gyro.canId);
    assertEquals(constants.driveConfig.canBus(), gyro.bus);
    assertEquals(
        Hertz.of(250.0), gyro.signalRates.rateFor("yawPosition", GyroConfig.CONTROL_FIELDS));
    assertEquals(
        SignalRates.DEFAULT.control(),
        gyro.signalRates.rateFor("pitchPosition", GyroConfig.CONTROL_FIELDS));
    assertEquals(0.1, constants.driverDeadband);
    assertEquals(0.1, constants.operatorDeadband);

    // Verify DriveConfig calculations
    double expectedRadius = Math.hypot(0.5, 0.5);
    assertEquals(expectedRadius, driveConfig.driveBaseRadius(), 1e-6);
    assertEquals(5.0 / expectedRadius, driveConfig.maxAngularVelocity(), 1e-6);

    Translation2d[] translations = driveConfig.getModuleTranslations();
    assertEquals(4, translations.length);
    assertEquals(new Translation2d(0.5, 0.5), translations[0]);
    assertEquals(new Translation2d(0.5, -0.5), translations[1]);
    assertEquals(new Translation2d(-0.5, 0.5), translations[2]);
    assertEquals(new Translation2d(-0.5, -0.5), translations[3]);

    SwerveDriveKinematics kinematics = driveConfig.kinematics();
    assertNotNull(kinematics);
    assertNotNull(driveConfig.toString());
    assertEquals(driveConfig, driveConfig);
    assertEquals(driveConfig.hashCode(), driveConfig.hashCode());
    assertNotNull(SwerveDriveConstants.DriveConfig.builder().toString());
    assertNotNull(autoAlignConstants.toString());
    assertNotNull(constants.toString());
  }
}
