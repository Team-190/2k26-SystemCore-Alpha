package edu.wpi.team190.gompeilib.subsystems.arm;

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
import java.util.Optional;
import java.util.function.Consumer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngleUnit;

public class ArmTest extends GompeiLibTestBase {
  private MotorIO motor;
  private MotorIOFactory factory;
  private MotorConfig builtConfig;
  private Consumer<MotorTelemetry> reading = t -> {};
  private Subsystem subsystem;
  private ArmConstants constants;
  private Gains slot0;

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
    when(subsystem.getName()).thenReturn("TestArmSubsystem");

    slot0 = Gains.builder().withPrefix("arm/slot0").withKG(0.3).build();
    constants =
        ArmConstants.builder()
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
                    .withSlot(GainSlot.ZERO, new SlotConfig(slot0, GravityType.COSINE))
                    .build())
            .withArmParameters(
                ArmConstants.ArmParameters.builder()
                    .withArmGearbox(DCMotor.getKrakenX60(1))
                    .withMinAngle(Rotation2d.fromDegrees(-90))
                    .withMaxAngle(Rotation2d.fromDegrees(90))
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
            .withPositionOffsetStep(Rotation2d.fromDegrees(5))
            .build();
  }

  private static OpenLoopRequest.VoltageOutput voltage(double volts) {
    return new OpenLoopRequest.VoltageOutput(Volts.of(volts), true, RequestOptions.DEFAULT);
  }

  @Test
  public void testAddsTheProfileFromItsConstraints() {
    new Arm(factory, subsystem, 0, constants);

    assertEquals(
        MotionProfileConfig.trapezoidal(
            RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0)),
        builtConfig.motionProfile);
  }

  @Test
  public void testPeriodicRunsTheCurrentState() {
    try (MockedStatic<Logger> logger = mockStatic(Logger.class)) {
      Arm arm = new Arm(factory, subsystem, 0, constants);
      assertEquals(ArmState.IDLE, arm.getCurrentState());

      arm.periodic();
      verify(motor).updateInputs(any());
      verify(motor, never()).setControl(any());
      logger.verify(() -> Logger.processInputs(eq("TestArmSubsystem/Arms0/Leader"), any()));

      arm.setVoltageGoal(Volts.of(4.0));
      arm.periodic();
      verify(motor).setControl(voltage(4.0));

      arm.setPositionGoal(Rotation2d.fromDegrees(45));
      arm.setGainSlot(GainSlot.TWO);
      arm.periodic();
      verify(motor)
          .setControl(
              new PositionRequest.ProfiledPositionVoltage(
                  Rotation2d.fromDegrees(45).getMeasure(),
                  Volts.of(0.0),
                  true,
                  GainSlot.TWO,
                  RequestOptions.DEFAULT));
      assertEquals(GainSlot.TWO, arm.getGainSlot());

      Setpoint<AngleUnit> goal =
          new Setpoint<>(Degrees.of(10), Degrees.of(1), Degrees.of(-90), Degrees.of(90));
      arm.setPositionGoal(goal);
      assertSame(goal, arm.getPositionGoal());
      arm.setVoltageGoal(new Setpoint<>(Volts.of(2.0), Volts.of(0.1), Volts.of(-12), Volts.of(12)));
      arm.periodic();
      verify(motor).setControl(voltage(2.0));
      assertNotNull(arm.getVoltageGoal());
    }
  }

  @Test
  public void testGoalsCompareAgainstTheLeader() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      Arm arm =
          new Arm(
              factory,
              subsystem,
              0,
              constants,
              new Setpoint<>(Degrees.of(30), Degrees.of(1), Degrees.of(-90), Degrees.of(90)));
      reading =
          t -> {
            t.position = Degrees.of(29.0);
            t.appliedVoltage = Volts.of(0.2);
          };
      arm.periodic();

      assertEquals(29.0, arm.getArmPosition().getDegrees(), 1e-9);
      assertTrue(arm.atPositionGoal());
      assertFalse(arm.atPositionGoal(Rotation2d.fromDegrees(40)));
      assertTrue(arm.atVoltageGoal());
      assertFalse(arm.atVoltageGoal(Volts.of(3.0)));

      Command wait = arm.waitUntilAtGoal();
      wait.initialize();
      assertTrue(wait.isFinished());

      arm.setPosition(Rotation2d.fromDegrees(12));
      verify(motor).setPosition(Rotation2d.fromDegrees(12).getMeasure());
    }
  }

  @Test
  public void testSeedsPositionFromItsEncoder() {
    try (MockedStatic<Logger> logger = mockStatic(Logger.class)) {
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
              1,
              constants,
              new Setpoint<>(Degrees.of(0), Degrees.of(1), Degrees.of(-90), Degrees.of(90)),
              new Setpoint<>(Volts.of(0), Volts.of(0.5), Volts.of(-12), Volts.of(12)));

      arm.periodic();
      logger.verify(() -> Logger.processInputs(eq("TestArmSubsystem/Arms1/Encoder"), any()));

      arm.resetPositionFromEncoder();
      verify(motor).setPosition(Rotations.of(0.125));
    }
  }

  @Test
  public void testResetFromEncoderWithoutOneDoesNothing() {
    Arm arm = new Arm(factory, subsystem, 0, constants);
    arm.resetPositionFromEncoder();
    verify(motor, never()).setPosition(any());
  }

  @Test
  public void testForwardsGainsAndConstraints() throws Exception {
    Arm arm = new Arm(factory, subsystem, 0, constants);

    Gains gains = Gains.builder().withKP(3.0).build();
    arm.updateGains(gains, GainSlot.ONE);
    verify(motor).updateGains(gains, GainSlot.ONE);

    arm.updateConstraints(
        AngularPositionConstraints.builder()
            .withMaxVelocity(RotationsPerSecond.of(0.5))
            .withMaxAcceleration(RotationsPerSecondPerSecond.of(0.5))
            .build());
    ArgumentCaptor<MotionProfileConfig> profile =
        ArgumentCaptor.forClass(MotionProfileConfig.class);
    verify(motor).updateMotionProfile(profile.capture());
    assertEquals(0.5, profile.getValue().cruiseVelocity().in(RotationsPerSecond), 1e-9);

    var lastNotified = Gains.class.getDeclaredField("lastNotifiedValues");
    lastNotified.setAccessible(true);
    lastNotified.set(slot0, null);
    var notify = Gains.class.getDeclaredMethod("notifyListeners");
    notify.setAccessible(true);
    notify.invoke(slot0);
    verify(motor).updateGains(slot0, GainSlot.ZERO);
  }

  @Test
  public void testSimMechanism() {
    assertInstanceOf(ArmMechanismSim.class, Arm.simMechanism(constants));
  }

  @Test
  public void testSysIdDrivesTheArm() throws Exception {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      Arm arm = new Arm(factory, subsystem, 0, constants);
      Command sysId = arm.sysIdRoutine();
      sysId.initialize();
      sysId.execute();
      sysId.end(true);
      assertEquals(ArmState.IDLE, arm.getCurrentState());

      var routineField = Arm.class.getDeclaredField("characterizationRoutine");
      routineField.setAccessible(true);
      Command quasistatic =
          ((SysIdRoutine) routineField.get(arm)).quasistatic(SysIdRoutine.Direction.FORWARD);
      quasistatic.initialize();
      quasistatic.execute();
      quasistatic.end(true);
      verify(motor, atLeastOnce()).setControl(isA(OpenLoopRequest.VoltageOutput.class));
    }
  }
}
