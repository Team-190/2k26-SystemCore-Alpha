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
import org.wpilib.units.CurrentUnit;

public class CurrentControllableTest extends GompeiLibTestBase {
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
  public void testDelegatesToTheCurrentControl() {
    mechanism.setCurrentGoal(Amps.of(8));
    assertEquals("Current", mechanism.getMode());
    assertEquals(8.0, mechanism.getCurrentGoal().getNewSetpoint().in(Amps), 1e-9);

    Setpoint<CurrentUnit> goal = new Setpoint<>(Amps.of(4), Amps.of(1));
    mechanism.setCurrentGoal(goal);
    assertSame(goal, mechanism.getCurrentGoal());

    motors.read(t -> t.torqueCurrent = Amps.of(4));
    assertTrue(mechanism.atCurrentGoal());
    assertFalse(mechanism.atCurrentGoal(Amps.of(9)));
  }
}
