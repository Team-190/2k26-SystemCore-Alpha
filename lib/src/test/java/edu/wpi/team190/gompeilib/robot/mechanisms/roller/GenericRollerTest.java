package edu.wpi.team190.gompeilib.robot.mechanisms.roller;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.FollowerMotor;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.physics.DCMotorMechanismSim;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControllable;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.VoltageUnit;

public class GenericRollerTest extends GompeiLibTestBase {
  private final List<MotorIO> motors = new ArrayList<>();
  private MotorIOFactory factory;
  private Subsystem subsystem;
  private GenericRollerConstants constants;

  @BeforeEach
  public void setUp() {
    factory = mock(MotorIOFactory.class);
    when(factory.family()).thenReturn("Mock");
    when(factory.supportsHardwareFollowing()).thenReturn(true);
    when(factory.create(any()))
        .thenAnswer(
            invocation -> {
              MotorIO motor = mock(MotorIO.class);
              motors.add(motor);
              return motor;
            });
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("TestRollerSubsystem");

    constants =
        GenericRollerConstants.builder()
            .withMotor(
                MotorConfig.builder()
                    .withCanId(1)
                    .withBus(CANBusId.named("rio"))
                    .withCurrentLimits(
                        CurrentLimits.fromDoubles()
                            .withSupplyCurrentLimit(30.0)
                            .withStatorCurrentLimit(30.0)
                            .build())
                    .withGearRatio(5.0)
                    .withEnableFOC(false)
                    .withFollower(new FollowerMotor(2, MotorAlignment.ALIGNED))
                    .build())
            .withRollerGearbox(DCMotor.getNeo550(2))
            .withMomentOfInertia(KilogramSquareMeters.of(0.001))
            .withVoltageOffsetStep(Volts.of(0.5))
            .build();
  }

  private static OpenLoopRequest.VoltageOutput voltage(double volts) {
    return new OpenLoopRequest.VoltageOutput(Volts.of(volts), false, RequestOptions.DEFAULT);
  }

  @Test
  public void testPeriodicSendsTheVoltageGoal() {
    try (MockedStatic<Logger> logger = mockStatic(Logger.class)) {
      GenericRoller roller = new GenericRoller(factory, subsystem, constants, "Test");
      MotorIO leader = motors.get(0);

      roller.periodic();
      verify(leader).updateInputs(any());
      verify(leader).setControl(voltage(0.0));
      logger.verify(() -> Logger.processInputs(eq("TestRollerSubsystem/Test/Leader"), any()));

      roller.setVoltageGoal(Volts.of(6.0));
      roller.periodic();
      verify(leader).setControl(voltage(6.0));

      Setpoint<VoltageUnit> setpoint =
          new Setpoint<>(Volts.of(4.0), Volts.of(0.5), Volts.of(-12.0), Volts.of(12.0));
      roller.setVoltageGoal(setpoint);
      roller.periodic();
      verify(leader).setControl(voltage(4.0));
      assertSame(setpoint, roller.getVoltageGoal());

      roller.setCurrentGoal(Amps.of(20.0));
      roller.periodic();
      verify(leader).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(20.0)));
      assertNotNull(roller.currentControl());

      roller.setDutyCycleGoal(0.8);
      roller.periodic();
      verify(leader)
          .setControl(new OpenLoopRequest.DutyCycleOutput(0.8, false, RequestOptions.DEFAULT));
      assertNotNull(roller.dutyCycleControl());
    }
  }

  @Test
  public void testFollowerHardwareFollows() {
    new GenericRoller(factory, subsystem, constants, "Test");

    assertEquals(2, motors.size());
    verify(motors.get(1)).setControl(any());
  }

  @Test
  public void testAtVoltageGoalUsesTheLeadersAppliedVoltage() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      GenericRoller roller =
          new GenericRoller(
              factory,
              subsystem,
              constants,
              "Test",
              new Setpoint<>(Volts.of(6.0), Volts.of(0.5), Volts.of(-12.0), Volts.of(12.0)));
      doAnswer(
              invocation -> {
                ((MotorTelemetry) invocation.getArgument(0)).appliedVoltage = Volts.of(5.8);
                return null;
              })
          .when(motors.get(0))
          .updateInputs(any());

      assertFalse(roller.atVoltageGoal());
      roller.periodic();
      assertTrue(roller.atVoltageGoal());
      assertTrue(roller.atVoltageGoal(Volts.of(6.2)));
      assertFalse(roller.atVoltageGoal(Volts.of(7.0)));

      assertTrue(roller.atCurrentGoal());
      assertFalse(roller.atCurrentGoal(Amps.of(10.0)));
      assertTrue(roller.atDutyCycleGoal());
      assertFalse(roller.atDutyCycleGoal(0.5));
    }
  }

  @Test
  public void testTorqueCurrentPerMotor() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      GenericRoller roller = new GenericRoller(factory, subsystem, constants, "Test");
      for (int i = 0; i < motors.size(); i++) {
        double amps = 10.0 * (i + 1);
        doAnswer(
                invocation -> {
                  ((MotorTelemetry) invocation.getArgument(0)).torqueCurrent = Amps.of(amps);
                  return null;
                })
            .when(motors.get(i))
            .updateInputs(any());
      }

      roller.periodic();

      assertEquals(List.of(Amps.of(10.0), Amps.of(20.0)), roller.getTorqueCurrents());
      assertEquals(Amps.of(10.0), roller.getTorqueCurrent());
    }
  }

  @Test
  public void testStartsDrivingItsVoltageGoal() {
    GenericRoller roller = new GenericRoller(factory, subsystem, constants, "Test");
    assertInstanceOf(VoltageControllable.class, roller);
    assertEquals("Voltage", roller.getMode());
    assertEquals("TestRollerSubsystem/Test", roller.core().getTopic());
    assertTrue(roller.core().isActive(roller.voltageControl()));
  }

  @Test
  public void testSimMechanism() {
    assertInstanceOf(DCMotorMechanismSim.class, GenericRoller.simMechanism(constants));
  }
}
