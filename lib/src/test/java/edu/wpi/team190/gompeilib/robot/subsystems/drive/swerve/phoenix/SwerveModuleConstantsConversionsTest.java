package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.encoder.config.SensorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.FeedbackSensor;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.NeutralMode;
import edu.wpi.team190.gompeilib.io.motor.config.RemoteSensorMode;
import edu.wpi.team190.gompeilib.io.motor.config.TorqueCurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveModuleConfig;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

public class SwerveModuleConstantsConversionsTest extends GompeiLibTestBase {
  private static final CANBusId BUS = CANBusId.named("drive");

  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants(boolean inverted) {
    var c =
        new SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>();
    c.DriveMotorId = 1;
    c.SteerMotorId = 2;
    c.EncoderId = 3;
    c.EncoderOffset = 0.25;
    c.LocationX = 0.3;
    c.LocationY = -0.4;
    c.DriveMotorInverted = inverted;
    c.SteerMotorInverted = inverted;
    c.EncoderInverted = inverted;
    c.DriveMotorGearRatio = 6.0;
    c.SteerMotorGearRatio = 20.0;
    c.SlipCurrent = 90.0;
    c.DriveInertia = 0.02;
    c.SteerInertia = 0.03;
    c.FeedbackSource = SteerFeedbackType.FusedCANcoder;
    c.DriveMotorGains = new Slot0Configs().withKP(1.0).withKV(0.5);
    c.SteerMotorGains = new Slot0Configs().withKP(50.0).withKD(0.5);
    return c;
  }

  @Test
  public void testConvertsTheDriveMotor() {
    SwerveModuleConfig config =
        SwerveModuleConstantsConversions.toModuleConfig(
            constants(false), BUS, Hertz.of(250.0), Seconds.of(0.02));
    MotorConfig drive = config.driveMotor();

    assertEquals(1, drive.canId);
    assertEquals(BUS, drive.bus);
    assertEquals(MotorDirection.COUNTER_CLOCKWISE_POSITIVE, drive.direction);
    assertEquals(NeutralMode.BRAKE, drive.neutralMode);
    assertNull(drive.currentLimits.supplyCurrentLimit());
    assertEquals(Amps.of(90.0), drive.currentLimits.statorCurrentLimit());
    assertEquals(TorqueCurrentLimits.symmetric(Amps.of(90.0)), drive.torqueCurrentLimits);
    assertEquals(6.0, drive.gearRatio);
    assertEquals(1.0, drive.slots.get(GainSlot.ZERO).gains().getKP());
    assertEquals(0.5, drive.slots.get(GainSlot.ZERO).gains().getKV());
    assertEquals(Seconds.of(0.02), drive.closedLoopRamps.current());
    assertEquals(
        Hertz.of(250.0), drive.signalRates.rateFor("position", MotorConfig.CONTROL_FIELDS));
  }

  @Test
  public void testConvertsTheTurnMotor() {
    SwerveModuleConfig config =
        SwerveModuleConstantsConversions.toModuleConfig(
            constants(true), BUS, Hertz.of(250.0), Seconds.of(0.02));
    MotorConfig turn = config.turnMotor();

    assertEquals(2, turn.canId);
    assertEquals(MotorDirection.CLOCKWISE_POSITIVE, turn.direction);
    assertEquals(Amps.of(40.0), turn.currentLimits.statorCurrentLimit());
    assertEquals(20.0, turn.gearRatio);
    assertEquals(new FeedbackSensor.RemoteEncoder(3, RemoteSensorMode.FUSED), turn.feedbackSensor);
    assertTrue(turn.continuousWrap);
    assertEquals(50.0, turn.slots.get(GainSlot.ZERO).gains().getKP());
    assertEquals(5.0, turn.motionProfile.cruiseVelocity().in(RotationsPerSecond), 1e-9);
    assertEquals(50.0, turn.motionProfile.acceleration().in(RotationsPerSecondPerSecond), 1e-9);
    assertEquals(2.4, turn.motionProfile.exponentialKV().in(Volts.per(RotationsPerSecond)), 1e-9);
    assertEquals(
        0.1, turn.motionProfile.exponentialKA().in(Volts.per(RotationsPerSecondPerSecond)), 1e-9);
  }

  @Test
  public void testConvertsTheEncoderAndGeometry() {
    var constants = constants(true);
    constants.EncoderInitialConfigs = new CANcoderConfiguration();
    constants.EncoderInitialConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
    SwerveModuleConfig config =
        SwerveModuleConstantsConversions.toModuleConfig(
            constants, BUS, Hertz.of(250.0), Seconds.of(0.02));
    AbsoluteEncoderConfig encoder = config.encoder();

    assertEquals(3, encoder.canId);
    assertEquals(BUS, encoder.bus);
    assertEquals(SensorDirection.CLOCKWISE_POSITIVE, encoder.direction);
    assertEquals(Rotations.of(0.25), encoder.magnetOffset);
    assertEquals(Rotations.of(1.0), encoder.discontinuityPoint);
    assertEquals(new Translation2d(0.3, -0.4), config.location());
    assertEquals(KilogramSquareMeters.of(0.02), config.driveInertia());
    assertEquals(KilogramSquareMeters.of(0.03), config.turnInertia());

    SwerveModuleConfig withoutInitialConfigs =
        SwerveModuleConstantsConversions.toModuleConfig(
            constants(false), BUS, Hertz.of(250.0), Seconds.of(0.02));
    assertEquals(
        SensorDirection.COUNTER_CLOCKWISE_POSITIVE, withoutInitialConfigs.encoder().direction);
    assertEquals(Rotations.of(0.5), withoutInitialConfigs.encoder().discontinuityPoint);
  }

  @Test
  public void testConvertsOutputTypes() {
    assertEquals(
        OutputType.VOLTAGE,
        SwerveModuleConstantsConversions.toOutputType(ClosedLoopOutputType.Voltage));
    assertEquals(
        OutputType.CURRENT,
        SwerveModuleConstantsConversions.toOutputType(ClosedLoopOutputType.TorqueCurrentFOC));
  }

  @Test
  public void testConvertsFeedbackSources() {
    assertEquals(
        RemoteSensorMode.FUSED,
        SwerveModuleConstantsConversions.toRemoteSensorMode(SteerFeedbackType.FusedCANcoder));
    assertEquals(
        RemoteSensorMode.SYNC,
        SwerveModuleConstantsConversions.toRemoteSensorMode(SteerFeedbackType.SyncCANcoder));
    assertEquals(
        RemoteSensorMode.REMOTE,
        SwerveModuleConstantsConversions.toRemoteSensorMode(SteerFeedbackType.RemoteCANcoder));
  }

  @Test
  public void shouldRejectFeedbackSourcesWithoutACANcoder() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            SwerveModuleConstantsConversions.toRemoteSensorMode(SteerFeedbackType.FusedCANdiPWM1));
  }

  @Test
  public void testConvertsGains() {
    Gains gains =
        SwerveModuleConstantsConversions.toGains(
            new Slot0Configs()
                .withKP(1)
                .withKI(2)
                .withKD(3)
                .withKS(4)
                .withKV(5)
                .withKA(6)
                .withKG(7));
    assertEquals(1, gains.getKP());
    assertEquals(2, gains.getKI());
    assertEquals(3, gains.getKD());
    assertEquals(4, gains.getKS());
    assertEquals(5, gains.getKV());
    assertEquals(6, gains.getKA());
    assertEquals(7, gains.getKG());
  }
}
