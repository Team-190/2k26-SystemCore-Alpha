package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.units.DimensionlessUnit;

public class DutyCycleControlTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private MechanismCore core;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    motors = new TestMotors();
    core = motors.core();
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  @Test
  public void testRegistersAndStartsAtZeroLimitedToOne() {
    DutyCycleControl control = new DutyCycleControl(core, 0.05, false);
    assertTrue(core.components().contains(control));
    assertEquals("DutyCycle", control.name());
    assertEquals(0.0, control.getGoal().getNewSetpoint().in(Value), 1e-9);

    control.getGoal().setSetpoint(Value.of(0.5));
    control.getGoal().increment(Value.of(2.0));
    assertEquals(0.5, control.getGoal().getOffset().in(Value), 1e-9);
  }

  @Test
  public void testRequestForAnyDutyCycleUsesTheControlsFoc() {
    DutyCycleControl control = new DutyCycleControl(core, 0.05, true);
    assertEquals(
        new OpenLoopRequest.DutyCycleOutput(0.75, true, RequestOptions.DEFAULT),
        control.request(0.75));
    assertEquals(
        new OpenLoopRequest.DutyCycleOutput(0.5, true, RequestOptions.DEFAULT),
        control.request(Value.of(0.5)));
    assertEquals(MechanismCore.IDLE, core.getMode());
  }

  @Test
  public void testSettingAGoalActivatesItAndSendsTheDutyCycle() {
    DutyCycleControl control = new DutyCycleControl(core, 0.05, false);

    control.setGoal(0.4);
    assertTrue(core.isActive(control));
    core.periodic();
    verify(motors.motor)
        .setControl(new OpenLoopRequest.DutyCycleOutput(0.4, false, RequestOptions.DEFAULT));

    control.setGoal(Value.of(0.6));
    assertEquals(0.6, control.getGoal().getSetpoint().in(Value), 1e-9);

    Setpoint<DimensionlessUnit> goal = new Setpoint<>(Value.of(0.2), Value.of(0.05));
    core.idle();
    control.setGoal(goal);
    assertSame(goal, control.getGoal());
    assertTrue(core.isActive(control));
  }

  @Test
  public void testAtGoalComparesTheDutyCycle() {
    DutyCycleControl control =
        new DutyCycleControl(core, new Setpoint<>(Value.of(0.5), Value.of(0.05)), true);
    motors.read(t -> t.dutyCycle = 0.52);

    assertTrue(control.atGoal());
    assertFalse(control.atGoal(0.8));
  }

  @Test
  public void testLogsUnderTheDutyCycleTopic() {
    DutyCycleControl control = new DutyCycleControl(core, 0.05, true);
    control.periodic("T");
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/DutyCycle/Goal"), any()));
    logger.verify(() -> Logger.recordOutputMeasure(eq("T/DutyCycle/Offset"), any()));
    logger.verify(() -> Logger.recordOutput("T/DutyCycle/AtGoal", true));
  }

  @Test
  public void shouldRequireAGoal() {
    assertThrows(
        NullPointerException.class,
        () -> new DutyCycleControl(core, (Setpoint<DimensionlessUnit>) null, true));
    DutyCycleControl control = new DutyCycleControl(core, 0.05, true);
    assertThrows(
        NullPointerException.class, () -> control.setGoal((Setpoint<DimensionlessUnit>) null));
    assertThrows(
        NullPointerException.class,
        () -> control.setGoal((org.wpilib.units.Measure<DimensionlessUnit>) null));
    assertThrows(
        NullPointerException.class,
        () -> control.request((org.wpilib.units.Measure<DimensionlessUnit>) null));
  }
}
