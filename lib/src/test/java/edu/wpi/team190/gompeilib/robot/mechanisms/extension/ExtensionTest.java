package edu.wpi.team190.gompeilib.robot.mechanisms.extension;

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
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import java.util.Optional;
import java.util.function.Consumer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;

public class ExtensionTest extends GompeiLibTestBase {
  private static final double DRUM_RADIUS = 0.02;

  private MockedStatic<Logger> logger;
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
    logger = mockStatic(Logger.class);
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
    constants = constants(Optional.empty());
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  private ExtensionConstants constants(Optional<CurrentHoming.Config<DistanceUnit>> homing) {
    return ExtensionConstants.builder()
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
        .withHoming(homing)
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
          t.closedLoopPositionReferenceSlope = RadiansPerSecond.of(metersPerSecond / DRUM_RADIUS);
        };
  }

  @Test
  public void testAddsSoftLimitsAndProfileInDrumRotations() {
    Extension extension = new Extension(factory, subsystem, "Extension", constants);

    assertEquals(
        SoftLimits.between(Radians.of(0.0), Radians.of(1.5 / DRUM_RADIUS)), builtConfig.softLimits);
    assertEquals(
        MotionProfileConfig.trapezoidal(
            RadiansPerSecond.of(2.0 / DRUM_RADIUS),
            RadiansPerSecondPerSecond.of(4.0 / DRUM_RADIUS)),
        builtConfig.motionProfile);
    assertEquals("TestExtensionSubsystem/Extension", extension.core().getTopic());
    assertEquals(MechanismCore.IDLE, extension.getMode());
    assertSame(constants, extension.getConstants());
  }

  @Test
  public void testSendsGoalsThroughTheDrum() {
    Extension extension = new Extension(factory, subsystem, "Extension", constants);

    extension.setPositionGoal(Meters.of(0.5));
    extension.periodic();
    verify(motor)
        .setControl(
            new PositionRequest.ProfiledPositionVoltage(
                Radians.of(0.5 / DRUM_RADIUS),
                Volts.of(0.0),
                true,
                GainSlot.ZERO,
                RequestOptions.DEFAULT));

    extension.setVoltageGoal(Volts.of(2.0));
    extension.periodic();
    verify(motor)
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(2.0), true, RequestOptions.DEFAULT));
  }

  @Test
  public void testReadsTravelThroughTheDrum() {
    Extension extension =
        new Extension(
            factory,
            subsystem,
            "Extension",
            constants,
            new Setpoint<>(Meters.of(0.75), Meters.of(0.05)));
    readAt(0.75, 0.4);
    extension.periodic();

    assertEquals(0.75, extension.getPosition().in(Meters), 1e-9);
    assertEquals(0.4, extension.getVelocity().in(MetersPerSecond), 1e-9);
    assertEquals(0.4, extension.getVelocitySetpoint().in(MetersPerSecond), 1e-9);
    assertTrue(extension.atPositionGoal());

    extension.setPosition(Meters.of(0.1));
    verify(motor).setPosition(Radians.of(0.1 / DRUM_RADIUS));
  }

  @Test
  public void testAtVoltageGoalComparesTheAppliedVoltage() {
    Extension extension =
        new Extension(
            factory,
            subsystem,
            "Extension",
            constants,
            new Setpoint<>(Meters.of(0), Meters.of(0.05)),
            new Setpoint<>(Volts.of(6.0), Volts.of(0.5)));
    reading = t -> t.appliedVoltage = Volts.of(0.0);
    extension.periodic();

    // The goal used to be compared against itself, so this was always true
    assertFalse(extension.atVoltageGoal());
  }

  @Test
  public void testForwardsGainsAndConstraints() throws Exception {
    Extension extension = new Extension(factory, subsystem, "Extension", constants);

    extension.updateConstraints(
        LinearConstraints.builder()
            .withMaxVelocity(MetersPerSecond.of(1.0))
            .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
            .build());
    verify(motor)
        .updateMotionProfile(
            MotionProfileConfig.trapezoidal(
                RadiansPerSecond.of(1.0 / DRUM_RADIUS),
                RadiansPerSecondPerSecond.of(1.0 / DRUM_RADIUS)));

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
  public void testHomesOnlyWithAHomingConfig() {
    assertThrows(
        IllegalStateException.class,
        () -> new Extension(factory, subsystem, "Extension", constants).home());

    Extension extension =
        new Extension(
            factory,
            subsystem,
            "Extension",
            constants(
                Optional.of(
                    new CurrentHoming.Config<>(
                        Volts.of(-1.0), Amps.of(-20.0), Amps.of(5.0), Meters.of(0.0)))));
    Command home = extension.home();
    home.initialize();
    home.execute();
    home.execute();
    verify(motor)
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(-1.0), true, CurrentHoming.OPTIONS));

    reading = t -> t.torqueCurrent = Amps.of(-20.0);
    extension.periodic();
    for (int i = 0; i < 5 && !home.isFinished(); i++) {
      home.execute();
    }
    assertTrue(home.isFinished());
    verify(motor).setPosition(Radians.of(0.0));
  }

  @Test
  public void testSysIdDrivesEveryExtension() throws Exception {
    Extension extension = new Extension(factory, subsystem, "Extension", constants);
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
            anyAtEndStop.invoke(null, SysIdRoutine.Direction.FORWARD, new Extension[] {extension}));
    assertFalse(
        (boolean)
            anyAtEndStop.invoke(null, SysIdRoutine.Direction.REVERSE, new Extension[] {extension}));
    readAt(0.0, 0.0);
    extension.periodic();
    assertTrue(
        (boolean)
            anyAtEndStop.invoke(null, SysIdRoutine.Direction.REVERSE, new Extension[] {extension}));

    // The sequence's idle step and end-stop checks are lambdas only a running scheduler calls.
    extension.setPositionGoal(Meters.of(0.5));
    for (var method : Extension.class.getDeclaredMethods()) {
      if (method.getName().startsWith("lambda$runSysIdRoutine$")) {
        method.setAccessible(true);
        method.invoke(null, (Object) new Extension[] {extension});
      }
    }
    assertEquals(MechanismCore.IDLE, extension.getMode());
  }
}
