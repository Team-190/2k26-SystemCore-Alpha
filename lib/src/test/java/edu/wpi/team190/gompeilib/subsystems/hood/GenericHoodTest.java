package edu.wpi.team190.gompeilib.subsystems.hood;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
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
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants;
import java.util.Optional;
import java.util.function.Consumer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;

public class GenericHoodTest extends GompeiLibTestBase {
  private MotorIO motor;
  private MotorIOFactory factory;
  private Consumer<MotorTelemetry> reading = t -> {};
  private Subsystem subsystem;
  private ArmConstants constants;

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
    when(factory.create(any())).thenReturn(motor);
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("TestHoodSubsystem");

    constants =
        ArmConstants.builder()
            .withMotor(
                MotorConfig.builder()
                    .withCanId(45)
                    .withBus(CANBusId.named("rio"))
                    .withCurrentLimits(
                        CurrentLimits.fromDoubles()
                            .withSupplyCurrentLimit(40.0)
                            .withStatorCurrentLimit(30.0)
                            .build())
                    .withGearRatio(70.0)
                    .withSlot(
                        GainSlot.ZERO,
                        new SlotConfig(Gains.builder().withPrefix("hood/slot0").build()))
                    .build())
            .withArmParameters(
                ArmConstants.ArmParameters.builder()
                    .withArmGearbox(DCMotor.getKrakenX44Foc(1))
                    .withMinAngle(Rotation2d.fromDegrees(0.5))
                    .withMaxAngle(Rotation2d.fromDegrees(35))
                    .withLength(Meters.of(0.2))
                    .withMomentOfInertia(KilogramSquareMeters.of(0.0001))
                    .withSimulateGravity(false)
                    .build())
            .withConstraints(
                AngularPositionConstraints.builder()
                    .withPrefix("hood/constraints")
                    .withMaxVelocity(RadiansPerSecond.of(37.5))
                    .withMaxAcceleration(RadiansPerSecondPerSecond.of(70))
                    .withGoalTolerance(Degrees.of(1))
                    .build())
            .withVoltageOffsetStep(Volts.of(0.5))
            .withPositionOffsetStep(Rotation2d.fromDegrees(1))
            .withHoming(
                Optional.of(
                    ArmConstants.Homing.builder()
                        .withVoltage(Volts.of(-1.0))
                        .withCurrentThreshold(Amps.of(-40.0))
                        .withCurrentEpsilon(Amps.of(0.5))
                        .withPosition(Rotation2d.ZERO)
                        .build()))
            .build();
  }

  private static OpenLoopRequest.VoltageOutput voltage(double volts) {
    return new OpenLoopRequest.VoltageOutput(Volts.of(volts), true, RequestOptions.DEFAULT);
  }

  @Test
  public void testRequiresAHomingConfig() {
    ArmConstants noHoming =
        ArmConstants.builder()
            .withMotor(constants.motor)
            .withArmParameters(constants.armParameters)
            .withConstraints(constants.constraints)
            .withVoltageOffsetStep(constants.voltageOffsetStep)
            .withPositionOffsetStep(constants.positionOffsetStep)
            .build();
    assertThrows(
        IllegalArgumentException.class, () -> new GenericHood(factory, subsystem, 0, noHoming));
  }

  @Test
  public void testDefaultGoalsAreLimitedToItsTravel() {
    GenericHood hood = new GenericHood(factory, subsystem, 0, constants);
    Setpoint<AngleUnit> positionGoal = hood.getPositionGoal();
    Setpoint<VoltageUnit> voltageGoal = hood.getVoltageGoal();

    // The bounds clamp the operator offset
    positionGoal.increment(Degrees.of(90));
    assertEquals(35.0, positionGoal.getOffset().in(Degrees), 1e-9);
    voltageGoal.increment(Volts.of(20));
    assertEquals(12.0, voltageGoal.getOffset().in(Volts), 1e-9);
  }

  @Test
  public void testRunsItsArm() {
    try (MockedStatic<Logger> logger = mockStatic(Logger.class)) {
      GenericHood hood = new GenericHood(factory, subsystem, 0, constants);

      hood.periodic();
      logger.verify(() -> Logger.processInputs(eq("TestHoodSubsystem/Arms0/Leader"), any()));
      verify(motor, never()).setControl(any());

      hood.setVoltageGoal(Volts.of(2.0));
      hood.periodic();
      verify(motor).setControl(voltage(2.0));

      hood.setVoltageGoal(new Setpoint<>(Volts.of(3.0), Volts.of(0.5)));
      hood.periodic();
      verify(motor).setControl(voltage(3.0));

      hood.setPositionGoal(Rotation2d.fromDegrees(20));
      hood.periodic();
      verify(motor)
          .setControl(
              new PositionRequest.ProfiledPositionVoltage(
                  Rotation2d.fromDegrees(20).getMeasure(),
                  Volts.of(0.0),
                  true,
                  GainSlot.ZERO,
                  RequestOptions.DEFAULT));

      Setpoint<AngleUnit> goal = new Setpoint<>(Degrees.of(10), Degrees.of(1));
      hood.setPositionGoal(goal);
      assertSame(goal, hood.getPositionGoal());
    }
  }

  @Test
  public void testReadsAndComparesAgainstItsArm() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      GenericHood hood =
          new GenericHood(
              factory, subsystem, 0, constants, new Setpoint<>(Degrees.of(20), Degrees.of(1)));
      reading =
          t -> {
            t.position = Degrees.of(19.5);
            t.velocity = RadiansPerSecond.of(1.5);
            t.appliedVoltage = Volts.of(0.1);
          };
      hood.periodic();

      assertEquals(19.5, hood.getAngle().getDegrees(), 1e-9);
      assertEquals(1.5, hood.getVelocity().in(RadiansPerSecond), 1e-9);
      assertTrue(hood.atPositionGoal());
      assertFalse(hood.atPositionGoal(Rotation2d.fromDegrees(30)));
      assertTrue(hood.atVoltageGoal());
      assertFalse(hood.atVoltageGoal(Volts.of(4.0)));
    }
  }

  @Test
  public void testWaitsALoopBeforeCheckingTheGoal() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      GenericHood hood =
          new GenericHood(
              factory, subsystem, 0, constants, new Setpoint<>(Degrees.of(0), Degrees.of(1)));
      hood.periodic();
      assertTrue(hood.atPositionGoal());

      Command wait = hood.waitUntilAtGoal();
      wait.initialize();
      assertFalse(wait.isFinished());
    }
  }

  @Test
  public void testForwardsGainsAndProfile() {
    GenericHood hood = new GenericHood(factory, subsystem, 0, constants);

    Gains gains = Gains.builder().withKP(5.0).build();
    hood.setGains(gains);
    verify(motor).updateGains(gains, GainSlot.ZERO);

    hood.setProfile(
        AngularPositionConstraints.builder()
            .withMaxVelocity(RotationsPerSecond.of(0.5))
            .withMaxAcceleration(RotationsPerSecondPerSecond.of(1.0))
            .build());
    verify(motor)
        .updateMotionProfile(
            MotionProfileConfig.trapezoidal(
                RotationsPerSecond.of(0.5), RotationsPerSecondPerSecond.of(1.0)));
  }

  @Test
  public void testResetHoodZeroHomesAgainstTheHardStop() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      GenericHood hood = new GenericHood(factory, subsystem, 0, constants);

      Command reset = hood.resetHoodZero();
      reset.initialize();
      reset.execute();
      reset.execute();
      verify(motor, atLeastOnce()).setControl(isA(OpenLoopRequest.VoltageOutput.class));

      reading = t -> t.torqueCurrent = Amps.of(-40.0);
      hood.periodic();
      for (int i = 0; i < 5 && !reset.isFinished(); i++) {
        reset.execute();
      }
      assertTrue(reset.isFinished());
      verify(motor).setPosition(Rotation2d.ZERO.getMeasure());
    }
  }

  @Test
  public void testSysIdAndSimMechanism() {
    GenericHood hood = new GenericHood(factory, subsystem, 0, constants);
    assertNotNull(hood.runSysIdRoutine());
    assertInstanceOf(ArmMechanismSim.class, GenericHood.simMechanism(constants));
  }
}
