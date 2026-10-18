package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.TestGains;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;
import org.wpilib.tunable.Tunable;
import org.wpilib.units.measure.Angle;

public class SwerveModuleTest extends GompeiLibTestBase {
  private static final double WHEEL_RADIUS = 0.05;

  private final List<MotorIO> motors = new ArrayList<>();
  private MotorIOFactory factory;
  private AbsoluteEncoderIO encoder;
  private boolean encoderConnected;

  @BeforeEach
  public void setUp() {
    motors.clear();
    encoderConnected = true;
    factory = mock(MotorIOFactory.class);
    when(factory.family()).thenReturn("Mock");
    when(factory.create(any()))
        .thenAnswer(
            invocation -> {
              MotorIO motor = mock(MotorIO.class);
              motors.add(motor);
              return motor;
            });
    encoder = mock(AbsoluteEncoderIO.class);
    doAnswer(
            invocation -> {
              ((AbsoluteEncoderTelemetry) invocation.getArgument(0)).connected = encoderConnected;
              return null;
            })
        .when(encoder)
        .updateInputs(any());
  }

  private static SwerveDriveConstants driveConstants(OutputType drive, OutputType steer) {
    var translation = SwerveDriveConstantsTest.translationGains(0, 0);
    var rotation = SwerveDriveConstantsTest.rotationGains(0, 0);
    SwerveModuleConfig module = SwerveDriveConstantsTest.createModuleConfig(1, 2, 3, 0.5, 0.5);
    LinearConstraints linear = LinearConstraints.builder().build();
    return SwerveDriveConstants.builder()
        .withDriveConfig(
            SwerveDriveConstants.DriveConfig.builder()
                .withCanBus(CANBusId.named("rio"))
                .withPigeon2Id(13)
                .withMaxLinearVelocityMetersPerSecond(5.0)
                .withWheelRadiusMeters(WHEEL_RADIUS)
                .withDriveModel(DCMotor.getKrakenX60(1))
                .withTurnModel(DCMotor.getKrakenX44(1))
                .withFrontLeft(module)
                .withFrontRight(module)
                .withBackLeft(module)
                .withBackRight(module)
                .withDriveClosedLoopOutputType(drive)
                .withSteerClosedLoopOutputType(steer)
                .withBumperWidth(0.8)
                .withBumperLength(0.8)
                .withRobotMassKilograms(45.0)
                .withTrackWidth(0.6)
                .withRobotMOI(2.5)
                .withModuleCurrentLimit(40.0)
                .withWheelCOF(1.2)
                .build())
        .withDriveGains(
            drive == OutputType.CURRENT
                ? Gains.angularVelocity(Amps).build()
                : Gains.angularVelocity(Volts).build())
        .withTurnGains(
            steer == OutputType.CURRENT
                ? Gains.angularPosition(Amps).build()
                : Gains.angularPosition(Volts).build())
        .withAutoTranslationGains(translation)
        .withAutoRotationGains(rotation)
        .withAutoAlignConstants(
            SwerveDriveConstants.AutoAlignConstants.builder()
                .withXGains(translation)
                .withXConstraints(linear)
                .withYGains(translation)
                .withYConstraints(linear)
                .withRotationGains(rotation)
                .withRotationConstraints(AngularPositionConstraints.builder().build())
                .withLinearThreshold(Tunable.create(Meters.of(0.01)))
                .withAngularThreshold(Tunable.create(Radians.of(0.01)))
                .build())
        .withOdometryFrequency(250.0)
        .withDriverDeadband(0.1)
        .withOperatorDeadband(0.1)
        .build();
  }

  private static void report(MotorIO motor, Angle position, double velocityRadPerSec, int samples) {
    doAnswer(
            invocation -> {
              MotorTelemetry t = invocation.getArgument(0);
              t.connected = true;
              t.position = position;
              t.velocity = RadiansPerSecond.of(velocityRadPerSec);
              t.odometryTimestamps = new double[samples];
              t.odometryPositions = new Angle[samples];
              for (int i = 0; i < samples; i++) {
                t.odometryTimestamps[i] = i;
                t.odometryPositions[i] = position;
              }
              return null;
            })
        .when(motor)
        .updateInputs(any());
  }

  private SwerveModule module(OutputType drive, OutputType steer) {
    return new SwerveModule(
        driveConstants(drive, steer),
        SwerveDriveConstantsTest.createModuleConfig(1, 2, 3, 0.5, 0.5, drive),
        new SwerveModuleHardware(factory, factory, encoder),
        0);
  }

  @Test
  public void testReadsMotorsAndEncoder() {
    try (MockedStatic<Logger> logger = mockStatic(Logger.class)) {
      SwerveModule module = module(OutputType.VOLTAGE, OutputType.VOLTAGE);
      MotorIO drive = motors.get(0);
      MotorIO turn = motors.get(1);
      report(drive, Radians.of(10.0), 4.0, 2);
      report(turn, Degrees.of(30.0), 0.0, 2);

      module.updateInputs();
      module.periodic();

      logger.verify(() -> Logger.processInputs(eq("Drive/Module0/Drive/Leader"), any()));
      logger.verify(() -> Logger.processInputs(eq("Drive/Module0/Turn/Leader"), any()));
      logger.verify(() -> Logger.processInputs(eq("Drive/Module0/Encoder"), any()));

      assertEquals(30.0, module.getAngle().getDegrees(), 1e-9);
      assertEquals(10.0 * WHEEL_RADIUS, module.getPositionMeters(), 1e-9);
      assertEquals(4.0 * WHEEL_RADIUS, module.getVelocityMetersPerSec(), 1e-9);
      assertEquals(10.0, module.getWheelRadiusCharacterizationPosition(), 1e-9);
      assertEquals(4.0 / (2 * Math.PI), module.getFFCharacterizationVelocity(), 1e-9);
      assertEquals(
          new SwerveModulePosition(10.0 * WHEEL_RADIUS, Rotation2d.fromDegrees(30.0)),
          module.getPosition());
      assertEquals(4.0 * WHEEL_RADIUS, module.getState().velocity, 1e-9);

      assertArrayEquals(new double[] {0.0, 1.0}, module.getOdometryTimestamps());
      assertEquals(2, module.getOdometryPositions().length);
      assertEquals(10.0 * WHEEL_RADIUS, module.getOdometryPositions()[1].distance, 1e-9);
      assertEquals(30.0, module.getOdometryPositions()[1].angle.getDegrees(), 1e-9);
    }
  }

  @Test
  public void testOdometryUsesTheFewestSamples() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      SwerveModule module = module(OutputType.VOLTAGE, OutputType.VOLTAGE);
      report(motors.get(0), Radians.of(1.0), 0.0, 3);
      report(motors.get(1), Radians.of(0.0), 0.0, 1);

      module.updateInputs();
      module.periodic();
      assertEquals(1, module.getOdometryPositions().length);

      report(motors.get(1), Radians.of(0.0), 0.0, 5);
      module.updateInputs();
      module.periodic();
      assertEquals(3, module.getOdometryPositions().length);
    }
  }

  @Test
  public void testRaisesAlertsForDisconnectedHardware() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      encoderConnected = false;
      SwerveModule module = module(OutputType.VOLTAGE, OutputType.VOLTAGE);
      module.updateInputs();
      assertDoesNotThrow(module::periodic);
    }
  }

  @Test
  public void testRunsVoltageSetpoints() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      SwerveModule module = module(OutputType.VOLTAGE, OutputType.VOLTAGE);
      MotorIO drive = motors.get(0);
      MotorIO turn = motors.get(1);

      SwerveModuleVelocity applied =
          module.runSetpoint(
              new SwerveModuleVelocity(1.0, Rotation2d.fromDegrees(0.0)),
              new SwerveModuleVelocity(6.0, Rotation2d.fromDegrees(0.0)));

      assertEquals(1.0, applied.velocity, 1e-9);
      DCMotor model = DCMotor.getKrakenX60(1);
      double amps = model.getCurrent(6.0 / 6.0);
      verify(drive)
          .setControl(
              new VelocityRequest.VelocityVoltage(
                  RadiansPerSecond.of(1.0 / WHEEL_RADIUS),
                  RadiansPerSecondPerSecond.of(0.0),
                  Volts.of(amps * model.R),
                  true,
                  GainSlot.ZERO,
                  RequestOptions.DEFAULT));
      verify(turn)
          .setControl(
              new PositionRequest.ProfiledPositionVoltage(
                  Rotation2d.fromDegrees(0.0).getMeasure(),
                  Volts.of(0.0),
                  true,
                  GainSlot.ZERO,
                  RequestOptions.DEFAULT));
    }
  }

  @Test
  public void testRunsCurrentSetpoints() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      SwerveModule module = module(OutputType.CURRENT, OutputType.CURRENT);
      MotorIO drive = motors.get(0);
      MotorIO turn = motors.get(1);

      module.runSetpoint(
          new SwerveModuleVelocity(1.0, Rotation2d.fromDegrees(0.0)),
          new SwerveModuleVelocity(0.0, Rotation2d.fromDegrees(0.0)));

      verify(drive)
          .setControl(
              new VelocityRequest.VelocityCurrent(
                  RadiansPerSecond.of(1.0 / WHEEL_RADIUS),
                  RadiansPerSecondPerSecond.of(0.0),
                  Amps.of(DCMotor.getKrakenX60(1).getCurrent(0.0)),
                  GainSlot.ZERO,
                  RequestOptions.DEFAULT));
      verify(turn)
          .setControl(
              new PositionRequest.ProfiledPositionCurrent(
                  Rotation2d.fromDegrees(0.0).getMeasure(),
                  Amps.of(0.0),
                  GainSlot.ZERO,
                  RequestOptions.DEFAULT));
    }
  }

  @Test
  public void testCharacterizationAndStop() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      SwerveModule module = module(OutputType.VOLTAGE, OutputType.VOLTAGE);
      MotorIO drive = motors.get(0);
      MotorIO turn = motors.get(1);

      module.runCharacterization(3.0);
      verify(drive).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(3.0)));
      verify(turn).setControl(isA(PositionRequest.ProfiledPositionVoltage.class));

      module.stop();
      verify(drive).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(0.0)));
      verify(turn).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(0.0)));
    }
  }

  @Test
  public void testUpdatesGainsAndCurrentLimits() {
    SwerveModule module = module(OutputType.VOLTAGE, OutputType.VOLTAGE);
    MotorIO drive = motors.get(0);
    MotorIO turn = motors.get(1);

    var driveGains = TestGains.velocity(1.0, 0, 0.1, 0.3, 0.4, 0, 0);
    module.updateDriveGains(driveGains, GainSlot.ZERO);
    verify(drive).updateGains(SlotConfig.velocity(driveGains), GainSlot.ZERO);
    verify(turn, never()).updateGains(any(), any());

    var turnGains = TestGains.position(2.0, 0, 0.2, 0, 0, 0, 0);
    module.updateTurnGains(turnGains, GainSlot.ONE);
    verify(turn).updateGains(SlotConfig.position(turnGains), GainSlot.ONE);

    module.updateCurrentLimits(50.0, 20.0);
    verify(drive).updateCurrentLimits(new CurrentLimits(Amps.of(60.0), Amps.of(50.0)));
    verify(turn).updateCurrentLimits(new CurrentLimits(Amps.of(60.0), Amps.of(20.0)));
  }

  @Test
  public void shouldRejectSlotsThatDontMatchTheModulesRequests() {
    MotorConfig voltageDrive =
        SwerveDriveConstantsTest.createModuleConfig(1, 2, 3, 0.5, 0.5).driveMotor();
    // A velocity slot where the module runs position, and voltage gains where it runs current
    assertThrows(
        IllegalArgumentException.class,
        () ->
            SwerveModule.requireSlot(
                voltageDrive, SlotConfig.Position.class, OutputType.VOLTAGE, "Turn"));
    assertThrows(
        IllegalArgumentException.class,
        () ->
            SwerveModule.requireSlot(
                voltageDrive, SlotConfig.Velocity.class, OutputType.CURRENT, "Drive"));
    SwerveModule.requireSlot(voltageDrive, SlotConfig.Velocity.class, OutputType.VOLTAGE, "Drive");
  }
}
