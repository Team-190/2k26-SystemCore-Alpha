package edu.wpi.team190.gompeilib.robot.mechanisms.extension;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.configs.SlotConfigs;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConstraint;
import edu.wpi.team190.gompeilib.characterization.CharacterizationRoutine;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.TestGains;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
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
import edu.wpi.team190.gompeilib.io.motor.phoenix.TalonFXConversions;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import edu.wpi.team190.gompeilib.io.physics.PlantMechanismSim;
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
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.VoltageUnit;

public class ExtensionTest extends GompeiLibTestBase {
  private static final double DRUM_RADIUS = 0.02;

  private MockedStatic<Logger> logger;
  private MotorIO motor;
  private MotorIOFactory factory;
  private MotorConfig builtConfig;
  private Consumer<MotorTelemetry> reading = t -> {};
  private Subsystem subsystem;
  private ExtensionConstants constants;
  private Gains<VoltageUnit, DistanceUnit, LinearVelocityUnit> slot0;
  private Gains<VoltageUnit, DistanceUnit, LinearVelocityUnit> slot1;

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

    slot0 =
        Gains.linearPosition(Volts)
            .withPrefix("slot0")
            .withKP(VoltsPerMeter.ofNative(10.0))
            .withKS(Volts.of(0.4))
            .withKV(VoltsPerMeterPerSecond.ofNative(6.0))
            .build();
    slot1 = Gains.linearPosition(Volts).withPrefix("slot1").build();
    constants = constants(Optional.empty());
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  private static MotorConfig.MotorConfigBuilder motor() {
    return MotorConfig.builder()
        .withCanId(5)
        .withBus(CANBusId.named("rio"))
        .withCurrentLimits(
            CurrentLimits.fromDoubles()
                .withSupplyCurrentLimit(40.0)
                .withStatorCurrentLimit(40.0)
                .build())
        .withGearRatio(10.0);
  }

  private ExtensionConstants constants(Optional<CurrentHoming.Config<DistanceUnit>> homing) {
    return constants(homing, motor().build());
  }

  private ExtensionConstants constants(
      Optional<CurrentHoming.Config<DistanceUnit>> homing, MotorConfig motor) {
    return ExtensionConstants.builder()
        .withMotor(motor)
        .withSlot(GainSlot.ZERO, slot0)
        .withSlot(GainSlot.ONE, slot1)
        .withDrumRadius(Meters.of(DRUM_RADIUS))
        .withHoming(homing)
        .withExtensionParameters(
            ExtensionConstants.ExtensionParameters.builder()
                .withCarriageGearbox(DCMotor.getNeo550(1))
                .withMinLength(Meters.of(0.0))
                .withMaxLength(Meters.of(1.5))
                .withPlant(
                    PlantCharacterization.ofElevator(
                            DCMotor.getNeo550(1), 10.0, Kilograms.of(15.0), Meters.of(0.02))
                        .withKG(Volts.of(0))
                        .build())
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
        .withCharacterization(
            CharacterizationConfig.voltage()
                .withRampRate(VoltsPerSecond.ofNative(1))
                .withStepOutput(Volts.of(3))
                .withQuasistaticTimeout(Seconds.of(3))
                .withDynamicTimeout(Seconds.of(3))
                .build())
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

    extension.setCurrentGoal(Amps.of(12.0));
    extension.periodic();
    verify(motor).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(12.0)));
    assertNotNull(extension.currentControl());

    extension.setDutyCycleGoal(0.4);
    extension.periodic();
    verify(motor)
        .setControl(new OpenLoopRequest.DutyCycleOutput(0.4, true, RequestOptions.DEFAULT));
    assertNotNull(extension.dutyCycleControl());
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
    verify(motor).updateGains(builtConfig.slots.get(GainSlot.ONE), GainSlot.ONE);
  }

  @Test
  public void shouldConvertGainsPerMeterThroughTheDrum() {
    new Extension(factory, subsystem, "Extension", constants);

    SlotConfig slot = builtConfig.slots.get(GainSlot.ZERO);
    assertInstanceOf(SlotConfig.Position.class, slot);
    SlotConfigs phoenix = TalonFXConversions.toSlotConfigs(slot, GainSlot.ZERO);
    double metersPerRotation = 2 * Math.PI * DRUM_RADIUS;
    assertEquals(10.0 * metersPerRotation, phoenix.kP, 1e-9);
    assertEquals(0.4, phoenix.kS, 1e-9);
    assertEquals(6.0 * metersPerRotation, phoenix.kV, 1e-9);
  }

  @Test
  public void shouldRejectGainsInTheMotorConfig() {
    ExtensionConstants withMotorSlots =
        constants(
            Optional.empty(),
            motor()
                .withSlot(GainSlot.ZERO, SlotConfig.position(TestGains.positionKP(1.0)))
                .build());
    assertThrows(
        IllegalArgumentException.class,
        () -> new Extension(factory, subsystem, "Extension", withMotorSlots));
  }

  @Test
  public void testSimMechanism() {
    assertInstanceOf(PlantMechanismSim.class, Extension.simMechanism(constants));
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
  public void testSysIdIdlesAndRampsTheExtension() {
    Extension extension = new Extension(factory, subsystem, "Extension", constants);
    extension.setPositionGoal(Meters.of(0.5));
    readAt(0.5, 0.0);
    extension.periodic();
    Command sysId = extension.characterize();
    sysId.initialize();
    sysId.execute();
    sysId.execute();
    assertEquals(MechanismCore.IDLE, extension.getMode());
    verify(motor, atLeastOnce()).setControl(isA(OpenLoopRequest.VoltageOutput.class));
    sysId.end(true);
    verify(motor, atLeastOnce())
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(0), true, RequestOptions.DEFAULT));
  }

  @Test
  public void testSysIdIsConstrainedToTheTravel() {
    Extension extension = new Extension(factory, subsystem, "Extension", constants);
    CharacterizationConstraint<?> constraint = extension.characterizationConstraint().orElseThrow();
    assertSame(constants.extensionParameters.minLength(), constraint.lowerLimit());
    assertSame(constants.extensionParameters.maxLength(), constraint.upperLimit());
    assertSame(constants.constraints.characterizationMargin(), constraint.margin());

    readAt(1.5, 0.0);
    extension.periodic();
    assertEquals(1.5, constraint.position().get().baseUnitMagnitude(), 1e-9);

    CharacterizationRoutine.MechanismTarget<VoltageUnit> target =
        (CharacterizationRoutine.MechanismTarget<VoltageUnit>)
            extension.characterizationTarget(
                CharacterizationConfig.voltage()
                    .withRampRate(VoltsPerSecond.ofNative(1))
                    .withStepOutput(Volts.of(3))
                    .withQuasistaticTimeout(Seconds.of(3))
                    .withDynamicTimeout(Seconds.of(3))
                    .build());
    assertSame(extension.core(), target.core());
    assertSame(constraint, target.constraint().orElseThrow());
    assertEquals(Optional.of(GravityType.CONSTANT), target.gravity());
    assertEquals(Optional.of(constants.drumRadius), target.drumRadius());
    assertEquals(
        new OpenLoopRequest.VoltageOutput(Volts.of(2), true, RequestOptions.DEFAULT),
        target.request().apply(Volts.of(2)));
  }

  @Test
  public void testRunsPositionWithCurrentOutputType() {
    ExtensionConstants currentConstants =
        constants.toBuilder()
            .withClosedLoopOutputType(edu.wpi.team190.gompeilib.io.motor.outputs.OutputType.CURRENT)
            .build();
    Extension extension = new Extension(factory, subsystem, "Extension", currentConstants);
    extension.setPositionGoal(Meters.of(0.5));
    extension.periodic();
    verify(motor)
        .setControl(
            new PositionRequest.ProfiledPositionCurrent(
                Radians.of(0.5 / DRUM_RADIUS),
                Amps.of(0.0),
                GainSlot.ZERO,
                RequestOptions.DEFAULT));
  }
}
