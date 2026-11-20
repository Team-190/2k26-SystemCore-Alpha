package edu.wpi.team190.gompeilib.robot.common.openloop;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class OpenLoopControlsTest extends GompeiLibTestBase {
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
  public void testBuildsAndRegistersTheThreeControls() {
    var voltageGoal = new Setpoint<>(Volts.of(0), Volts.of(1));
    OpenLoopControls controls =
        OpenLoopControls.create(
            core, MechanismTestSupport.config(), voltageGoal, Amps.of(2), Value.of(0.1));
    assertSame(voltageGoal, controls.voltage().getGoal());
    assertTrue(core.components().contains(controls.voltage()));
    assertTrue(core.components().contains(controls.current()));
    assertTrue(core.components().contains(controls.dutyCycle()));

    controls.dutyCycle().getGoal().increment();
    assertEquals(0.1, controls.dutyCycle().getGoal().getOffset().in(Value), 1e-9);
  }

  @Test
  public void testLimitsTheCurrentOffsetToTheStatorLimit() {
    OpenLoopControls controls =
        OpenLoopControls.create(
            core,
            MechanismTestSupport.config(),
            new Setpoint<>(Volts.of(0), Volts.of(1)),
            Amps.of(1),
            Value.of(0.05));
    controls.current().getGoal().increment(Amps.of(100));
    assertEquals(40.0, controls.current().getGoal().getOffset().in(Amps), 1e-9);
    controls.current().getGoal().decrement(Amps.of(200));
    assertEquals(-40.0, controls.current().getGoal().getOffset().in(Amps), 1e-9);
  }

  @Test
  public void testLeavesTheCurrentGoalUnclampedWithoutAStatorLimit() {
    MotorConfig motor =
        MechanismTestSupport.config().toBuilder()
            .withCurrentLimits(new CurrentLimits(Amps.of(40), null))
            .build();
    OpenLoopControls controls =
        OpenLoopControls.create(
            core, motor, new Setpoint<>(Volts.of(0), Volts.of(1)), Amps.of(1), Value.of(0.05));
    controls.current().getGoal().increment(Amps.of(100));
    assertEquals(100.0, controls.current().getGoal().getOffset().in(Amps), 1e-9);
  }

  @Test
  public void testUsesTheMotorsFoc() {
    MotorConfig motor = MechanismTestSupport.config().toBuilder().withEnableFOC(false).build();
    OpenLoopControls controls =
        OpenLoopControls.create(
            core, motor, new Setpoint<>(Volts.of(0), Volts.of(1)), Amps.of(1), Value.of(0.05));
    assertEquals(
        new OpenLoopRequest.VoltageOutput(Volts.of(1), false, RequestOptions.DEFAULT),
        controls.voltage().request(Volts.of(1)));
    assertEquals(
        new OpenLoopRequest.DutyCycleOutput(0.5, false, RequestOptions.DEFAULT),
        controls.dutyCycle().request(Value.of(0.5)));
  }

  @Test
  public void shouldRequireEveryControl() {
    OpenLoopControls controls =
        OpenLoopControls.create(
            core,
            MechanismTestSupport.config(),
            new Setpoint<>(Volts.of(0), Volts.of(1)),
            Amps.of(1),
            Value.of(0.05));
    assertThrows(
        NullPointerException.class,
        () -> new OpenLoopControls(null, controls.current(), controls.dutyCycle()));
    assertThrows(
        NullPointerException.class,
        () -> new OpenLoopControls(controls.voltage(), null, controls.dutyCycle()));
    assertThrows(
        NullPointerException.class,
        () -> new OpenLoopControls(controls.voltage(), controls.current(), null));
  }
}
