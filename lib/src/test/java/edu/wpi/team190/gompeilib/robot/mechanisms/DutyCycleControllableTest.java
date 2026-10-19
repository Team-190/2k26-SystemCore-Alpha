package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.units.DimensionlessUnit;

public class DutyCycleControllableTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private TestMechanism mechanism;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    motors = new TestMotors();
    mechanism = new TestMechanism(motors);
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  @Test
  public void testDelegatesToTheDutyCycleControl() {
    mechanism.setDutyCycleGoal(0.3);
    assertEquals("DutyCycle", mechanism.getMode());
    assertEquals(0.3, mechanism.getDutyCycleGoal().getNewSetpoint().in(Value), 1e-9);

    mechanism.setDutyCycleGoal(Value.of(0.4));
    assertEquals(0.4, mechanism.getDutyCycleGoal().getNewSetpoint().in(Value), 1e-9);

    Setpoint<DimensionlessUnit> goal = new Setpoint<>(Value.of(0.2), Value.of(0.05));
    mechanism.setDutyCycleGoal(goal);
    assertSame(goal, mechanism.getDutyCycleGoal());

    motors.read(t -> t.dutyCycle = 0.2);
    assertTrue(mechanism.atDutyCycleGoal());
    assertFalse(mechanism.atDutyCycleGoal(0.6));
  }
}
