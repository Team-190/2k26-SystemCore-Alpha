package edu.wpi.team190.gompeilib.robot.mechanisms.arm;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.physics.ArmMechanismSim;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import edu.wpi.team190.gompeilib.sysid.SysIdConfig;
import edu.wpi.team190.gompeilib.sysid.SysIdConstraint;
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
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;

public class ArmTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private MotorIO motor;
  private MotorIOFactory factory;
  private MotorConfig builtConfig;
  private Consumer<MotorTelemetry> reading = t -> {};
  private Subsystem subsystem;
  private ArmConstants constants;
  private Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> slot0;

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
    when(subsystem.getName()).thenReturn("TestArmSubsystem");

    slot0 = Gains.angularPosition(Volts).withPrefix("arm/slot0").withKG(Volts.of(0.3)).build();
    constants = constants(Optional.empty());
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  private ArmConstants constants(Optional<CurrentHoming.Config<AngleUnit>> homing) {
    return ArmConstants.builder()
        .withMotor(
            MotorConfig.builder()
                .withCanId(10)
                .withBus(CANBusId.named("rio"))
                .withCurrentLimits(
                    CurrentLimits.fromDoubles()
                        .withSupplyCurrentLimit(40.0)
                        .withStatorCurrentLimit(40.0)
                        .build())
                .withGearRatio(50.0)
                .withSlot(GainSlot.ZERO, SlotConfig.position(slot0, GravityType.COSINE))
                .build())
        .withArmParameters(
            ArmConstants.ArmParameters.builder()
                .withArmGearbox(DCMotor.getKrakenX60(1))
                .withMinAngle(Degrees.of(-90))
                .withMaxAngle(Degrees.of(90))
                .withLength(Meters.of(0.5))
                .withMomentOfInertia(KilogramSquareMeters.of(0.1))
                .withSimulateGravity(true)
                .build())
        .withConstraints(
            AngularPositionConstraints.builder()
                .withPrefix("arm/constraints")
                .withMaxVelocity(RotationsPerSecond.of(1.0))
                .withMaxAcceleration(RotationsPerSecondPerSecond.of(2.0))
                .withGoalTolerance(Degrees.of(2.0))
                .build())
        .withVoltageOffsetStep(Volts.of(0.5))
        .withPositionOffsetStep(Degrees.of(5))
        .withSysId(
            SysIdConfig.voltage()
                .withRampRate(CustomUnits.voltsPerSecond.ofNative(1))
                .withStepOutput(Volts.of(9))
                .withQuasistaticTimeout(Seconds.of(12))
                .withDynamicTimeout(Seconds.of(12))
                .build())
        .withHoming(homing)
        .build();
  }

  @Test
  public void testBuildsMotorsWithTheProfileAndStartsIdle() {
    Arm arm = new Arm(factory, subsystem, "Arm", constants);

    assertEquals(
        MotionProfileConfig.trapezoidal(
            RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)),
        builtConfig.motionProfile);
    assertEquals("TestArmSubsystem/Arm", arm.core().getTopic());
    assertEquals(MechanismCore.IDLE, arm.getMode());
    assertSame(arm.conversion(), arm.conversion());
  }

  @Test
  public void testRunsPositionAndVoltageGoals() {
    Arm arm = new Arm(factory, subsystem, "Arm", constants);

    arm.setPositionGoal(Degrees.of(45));
    arm.setPositionGainSlot(GainSlot.TWO);
    arm.periodic();
    verify(motor)
        .setControl(
            new PositionRequest.ProfiledPositionVoltage(
                Degrees.of(45), Volts.of(0.0), true, GainSlot.TWO, RequestOptions.DEFAULT));
    logger.verify(() -> Logger.processInputs(eq("TestArmSubsystem/Arm/Leader"), any()));

    arm.setVoltageGoal(Volts.of(4.0));
    arm.periodic();
    verify(motor)
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(4.0), true, RequestOptions.DEFAULT));
  }

  @Test
  public void testGoalsUseTheConstraintsTolerance() {
    Arm arm =
        new Arm(
            factory,
            subsystem,
            "Arm",
            constants,
            new Setpoint<>(Degrees.of(30), Degrees.of(1), Degrees.of(-90), Degrees.of(90)));
    reading = t -> t.position = Degrees.of(29.0);
    arm.periodic();

    assertEquals(29.0, arm.getPosition().in(Degrees), 1e-9);
    assertTrue(arm.atPositionGoal());
    assertFalse(arm.atPositionGoal(Degrees.of(40)));
  }

  @Test
  public void testSeedsPositionFromItsEncoder() {
    AbsoluteEncoderIO encoder = mock(AbsoluteEncoderIO.class);
    doAnswer(
            invocation -> {
              ((AbsoluteEncoderTelemetry) invocation.getArgument(0)).absolutePosition =
                  Rotations.of(0.125);
              return null;
            })
        .when(encoder)
        .updateInputs(any());
    Arm arm =
        new Arm(
            factory,
            Optional.of(encoder),
            subsystem,
            "Arm",
            constants,
            new Setpoint<>(Degrees.of(0), Degrees.of(1)),
            new Setpoint<>(Volts.of(0), Volts.of(0.5)));
    assertTrue(arm.encoderSeed().isPresent());

    arm.periodic();
    arm.seedPositionFromEncoder();
    verify(motor).setPosition(Rotations.of(0.125));
  }

  @Test
  public void testHomesOnlyWithAHomingConfig() {
    assertThrows(
        IllegalStateException.class, () -> new Arm(factory, subsystem, "Arm", constants).home());

    Arm arm =
        new Arm(
            factory,
            subsystem,
            "Arm",
            constants(
                Optional.of(
                    new CurrentHoming.Config<>(
                        Volts.of(-1), Amps.of(-20), Amps.of(1), Degrees.of(-90)))));
    Command home = arm.home();
    home.initialize();
    home.execute();
    home.execute();
    verify(motor)
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(-1), true, CurrentHoming.OPTIONS));
    assertEquals(Degrees.of(-90), arm.homing().getConfig().position());
  }

  @Test
  public void testForwardsGainAndConstraintChanges() throws Exception {
    Arm arm = new Arm(factory, subsystem, "Arm", constants);

    arm.updateConstraints(
        AngularPositionConstraints.builder()
            .withMaxVelocity(RotationsPerSecond.of(0.5))
            .withMaxAcceleration(RotationsPerSecondPerSecond.of(0.5))
            .build());
    verify(motor)
        .updateMotionProfile(
            MotionProfileConfig.trapezoidal(
                RotationsPerSecond.of(0.5), RotationsPerSecondPerSecond.of(0.5)));

    var lastNotified = Gains.class.getDeclaredField("lastNotifiedValues");
    lastNotified.setAccessible(true);
    lastNotified.set(slot0, null);
    var notify = Gains.class.getDeclaredMethod("notifyListeners");
    notify.setAccessible(true);
    notify.invoke(slot0);
    verify(motor).updateGains(SlotConfig.position(slot0, GravityType.COSINE), GainSlot.ZERO);

    var constraintsNotified =
        AngularPositionConstraints.class.getDeclaredField("lastNotifiedValues");
    constraintsNotified.setAccessible(true);
    constraintsNotified.set(constants.constraints, null);
    var constraintsNotify = AngularPositionConstraints.class.getDeclaredMethod("notifyListeners");
    constraintsNotify.setAccessible(true);
    constraintsNotify.invoke(constants.constraints);
    verify(motor)
        .updateMotionProfile(
            MotionProfileConfig.trapezoidal(
                RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)));
  }

  @Test
  public void testSimMechanism() {
    assertInstanceOf(ArmMechanismSim.class, Arm.simMechanism(constants));
  }

  @Test
  public void testSysIdIdlesAndRampsTheArm() {
    Arm arm = new Arm(factory, subsystem, "Arm", constants);
    arm.setPositionGoal(Degrees.of(10));
    Command sysId = arm.sysIdRoutine();
    sysId.initialize();
    sysId.execute();
    sysId.execute();
    assertEquals(MechanismCore.IDLE, arm.getMode());
    verify(motor, atLeastOnce()).setControl(isA(OpenLoopRequest.VoltageOutput.class));
    sysId.end(true);
    verify(motor, atLeastOnce())
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(0), true, RequestOptions.DEFAULT));
  }

  @Test
  public void testSysIdIsConstrainedToTheTravel() {
    Arm arm = new Arm(factory, subsystem, "Arm", constants);
    SysIdConstraint<?> constraint = arm.sysIdConstraint().orElseThrow();
    assertSame(constants.armParameters.minAngle(), constraint.lowerLimit());
    assertSame(constants.armParameters.maxAngle(), constraint.upperLimit());
    assertSame(constants.constraints.sysIdMargin(), constraint.margin());

    reading = t -> t.position = Rotations.of(0.25);
    arm.periodic();
    assertEquals(Math.PI / 2, constraint.position().get().baseUnitMagnitude(), 1e-9);
    assertTrue(constraint.atLimit(true));
    assertFalse(constraint.atLimit(false));
  }
}
