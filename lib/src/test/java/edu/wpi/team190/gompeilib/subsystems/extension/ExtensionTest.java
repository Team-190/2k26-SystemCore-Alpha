package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SoftLimits;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.physics.ElevatorMechanismSim;
import java.util.function.Consumer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;

public class ExtensionTest extends GompeiLibTestBase {
  private static final double DRUM_RADIUS = 0.02;

  private MotorIO motor;
  private MotorIOFactory factory;
  private MotorConfig builtConfig;
  private Consumer<MotorTelemetry> reading = t -> {};
  private Subsystem subsystem;
  private ExtensionConstants constants;
  private Gains slot0;
  private Gains slot1;

  @BeforeEach
  public void setUp() {
    motor = mock(MotorIO.class);
    doAnswer(
            invocation -> {
              reading.accept(invocation.getArgument(0));
              return null;
            })
        .when(motor)
        .updateInputs(any());
    factory = mock(MotorIOFactory.class);
    when(factory.family()).thenReturn("Mock");
    when(factory.create(any()))
        .thenAnswer(
            invocation -> {
              builtConfig = invocation.getArgument(0);
              return motor;
            });
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("TestExtensionSubsystem");

    slot0 = Gains.builder().withPrefix("slot0").build();
    slot1 = Gains.builder().withPrefix("slot1").build();
    constants =
        ExtensionConstants.builder()
            .withMotor(
                MotorConfig.builder()
                    .withCanId(5)
                    .withBus(CANBusId.named("rio"))
                    .withCurrentLimits(
                        CurrentLimits.fromDoubles()
                            .withSupplyCurrentLimit(40.0)
                            .withStatorCurrentLimit(40.0)
                            .build())
                    .withGearRatio(10.0)
                    .withSlot(GainSlot.ZERO, new SlotConfig(slot0))
                    .withSlot(GainSlot.ONE, new SlotConfig(slot1))
                    .build())
            .withDrumRadius(Meters.of(DRUM_RADIUS))
            .withExtensionParameters(
                ExtensionConstants.ExtensionParameters.builder()
                    .withCarriageGearbox(DCMotor.getNeo550(1))
                    .withCarriageMass(Kilograms.of(15.0))
                    .withMinLength(Meters.of(0.0))
                    .withMaxLength(Meters.of(1.5))
                    .withSimulateGravity(false)
                    .build())
            .withConstraints(
                LinearConstraints.builder()
                    .withPrefix("constraints")
                    .withMaxVelocity(MetersPerSecond.of(2.0))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(4.0))
                    .withGoalTolerance(Meters.of(0.05))
                    .build())
            .withVoltageOffsetStep(Volts.of(0.5))
            .withHeightOffsetStep(Meters.of(0.05))
            .build();
  }

  /** Reports the leader at {@code meters} of travel, moving at {@code metersPerSecond}. */
  private void readAt(double meters, double metersPerSecond) {
    reading =
        t -> {
          t.position = Radians.of(meters / DRUM_RADIUS);
          t.velocity = RadiansPerSecond.of(metersPerSecond / DRUM_RADIUS);
        };
  }

  private static OpenLoopRequest.VoltageOutput voltage(double volts) {
    return new OpenLoopRequest.VoltageOutput(Volts.of(volts), true, RequestOptions.DEFAULT);
  }

  @Test
  public void testAddsSoftLimitsAndProfileInDrumRotations() {
    new Extension(constants, subsystem, 0, factory);

    assertEquals(
        SoftLimits.between(Radians.of(0.0), Radians.of(1.5 / DRUM_RADIUS)), builtConfig.softLimits);
    assertEquals(
        2.0 / DRUM_RADIUS, builtConfig.motionProfile.cruiseVelocity().in(RadiansPerSecond), 1e-9);
    assertEquals(
        4.0 / DRUM_RADIUS,
        builtConfig.motionProfile.acceleration().in(RadiansPerSecondPerSecond),
        1e-9);
  }

  @Test
  public void testPeriodicRunsTheCurrentState() {
    try (MockedStatic<Logger> logger = mockStatic(Logger.class)) {
      Extension extension = new Extension(constants, subsystem, 0, factory);

      extension.periodic();
      verify(motor).updateInputs(any());
      verify(motor, never()).setControl(any());
      logger.verify(
          () -> Logger.processInputs(eq("TestExtensionSubsystem/Extension0/Leader"), any()));

      extension.setVoltageGoal(Volts.of(6.0));
      extension.periodic();
      verify(motor).setControl(voltage(6.0));

      extension.setPositionGoal(Meters.of(1.0));
      extension.setGainSlot(GainSlot.ONE);
      extension.periodic();
      verify(motor)
          .setControl(
              new PositionRequest.ProfiledPositionVoltage(
                  Radians.of(1.0 / DRUM_RADIUS),
                  Volts.of(0.0),
                  true,
                  GainSlot.ONE,
                  RequestOptions.DEFAULT));
      assertEquals(GainSlot.ONE, extension.getGainSlot());

      Setpoint<DistanceUnit> goal =
          new Setpoint<>(Meters.of(0.5), Meters.of(0.01), Meters.of(0.0), Meters.of(1.5));
      extension.setPositionGoal(goal);
      assertSame(goal, extension.getPositionGoal());
      extension.setVoltageGoal(
          new Setpoint<>(Volts.of(2.0), Volts.of(0.1), Volts.of(-12), Volts.of(12)));
      extension.periodic();
      verify(motor).setControl(voltage(2.0));
      assertTrue(extension.atVoltageGoal());
      assertFalse(extension.atVoltageGoal(Volts.of(3.0)));
    }
  }

  @Test
  public void testConvertsTelemetryThroughTheDrum() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      Extension extension =
          new Extension(
              constants,
              subsystem,
              0,
              factory,
              new Setpoint<>(Meters.of(0.5), Meters.of(0.01), Meters.of(0.0), Meters.of(1.5)));
      readAt(0.48, 0.3);
      reading =
          reading.andThen(
              t -> {
                t.torqueCurrent = Amps.of(12.0);
                t.closedLoopPositionReferenceSlope = RadiansPerSecond.of(0.1 / DRUM_RADIUS);
              });
      extension.periodic();

      assertEquals(0.48, extension.getExtensionPosition().in(Meters), 1e-9);
      assertEquals(0.3, extension.getVelocity().in(MetersPerSecond), 1e-9);
      assertEquals(0.1, extension.getVelocitySetpoint().in(MetersPerSecond), 1e-9);
      assertEquals(Amps.of(12.0), extension.getTorqueCurrent());
      assertTrue(extension.atPositionGoal());
      assertFalse(extension.atPositionGoal(Meters.of(1.0)));

      extension.setPosition(Meters.of(0.4));
      verify(motor).setPosition(Radians.of(0.4 / DRUM_RADIUS));

      Command wait = extension.waitUntilAtGoal();
      wait.initialize();
      assertTrue(wait.isFinished());
    }
  }

  @Test
  public void testForwardsGainsAndConstraints() throws Exception {
    Extension extension = new Extension(constants, subsystem, 0, factory);

    Gains gains = Gains.builder().withKP(3.0).build();
    extension.updateGains(gains, GainSlot.TWO);
    verify(motor).updateGains(gains, GainSlot.TWO);

    extension.updateConstraints(
        LinearConstraints.builder()
            .withMaxVelocity(MetersPerSecond.of(1.0))
            .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
            .build());
    ArgumentCaptor<MotionProfileConfig> profile =
        ArgumentCaptor.forClass(MotionProfileConfig.class);
    verify(motor).updateMotionProfile(profile.capture());
    assertEquals(1.0 / DRUM_RADIUS, profile.getValue().cruiseVelocity().in(RadiansPerSecond), 1e-9);

    // Tuning a slot's gains from a dashboard sends them to that slot.
    var lastNotified = Gains.class.getDeclaredField("lastNotifiedValues");
    lastNotified.setAccessible(true);
    lastNotified.set(slot1, null);
    var notify = Gains.class.getDeclaredMethod("notifyListeners");
    notify.setAccessible(true);
    notify.invoke(slot1);
    verify(motor).updateGains(slot1, GainSlot.ONE);
  }

  @Test
  public void testSimMechanism() {
    assertInstanceOf(ElevatorMechanismSim.class, Extension.simMechanism(constants));
  }

  @Test
  public void testZeroingDrivesToTheHardStop() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      Extension extension = new Extension(constants, subsystem, 0, factory);
      reading = t -> t.torqueCurrent = Amps.of(0.0);
      extension.periodic();

      Command reset = extension.resetExtensionZero();
      reset.initialize();
      reset.execute();
      reset.execute();
      verify(motor, atLeastOnce()).setControl(voltage(-1.0));

      reading = t -> t.torqueCurrent = Amps.of(-20.0);
      extension.periodic();
      for (int i = 0; i < 5 && !reset.isFinished(); i++) {
        reset.execute();
      }
      assertTrue(reset.isFinished());
      verify(motor).setControl(voltage(0.0));
      verify(motor).setPosition(Radians.of(0.0));
    }
  }

  @Test
  public void testSysIdDrivesEveryExtension() throws Exception {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      Extension extension = new Extension(constants, subsystem, 0, factory);
      assertNotNull(extension.runSysIdRoutine());
      assertNotNull(Extension.runSysIdRoutine(subsystem, extension));

      var routineField = Extension.class.getDeclaredField("characterizationRoutine");
      routineField.setAccessible(true);
      Command quasistatic =
          ((SysIdRoutine) routineField.get(extension)).quasistatic(SysIdRoutine.Direction.FORWARD);
      quasistatic.initialize();
      quasistatic.execute();
      quasistatic.end(true);
      verify(motor, atLeastOnce()).setControl(isA(OpenLoopRequest.VoltageOutput.class));

      var anyAtEndStop =
          Extension.class.getDeclaredMethod(
              "anyAtEndStop", SysIdRoutine.Direction.class, Extension[].class);
      anyAtEndStop.setAccessible(true);
      readAt(1.5, 0.0);
      extension.periodic();
      assertTrue(
          (boolean)
              anyAtEndStop.invoke(
                  null, SysIdRoutine.Direction.FORWARD, new Extension[] {extension}));
      assertFalse(
          (boolean)
              anyAtEndStop.invoke(
                  null, SysIdRoutine.Direction.REVERSE, new Extension[] {extension}));
      readAt(0.0, 0.0);
      extension.periodic();
      assertTrue(
          (boolean)
              anyAtEndStop.invoke(
                  null, SysIdRoutine.Direction.REVERSE, new Extension[] {extension}));

      // The sequence's state reset and end-stop checks are lambdas only a running scheduler calls.
      for (var method : Extension.class.getDeclaredMethods()) {
        if (method.getName().startsWith("lambda$runSysIdRoutine$")) {
          method.setAccessible(true);
          method.invoke(null, (Object) new Extension[] {extension});
        }
      }
    }
  }
}
