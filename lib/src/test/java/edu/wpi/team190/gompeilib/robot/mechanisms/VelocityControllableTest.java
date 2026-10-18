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
import org.wpilib.command2.Command;
import org.wpilib.units.AngularVelocityUnit;

public class VelocityControllableTest extends GompeiLibTestBase {
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
  public void testDelegatesToTheVelocityControl() {
    mechanism.setVelocityGoal(RadiansPerSecond.of(50));
    assertEquals("VelocityVoltage", mechanism.getMode());
    assertEquals(50.0, mechanism.getVelocityGoal().getNewSetpoint().in(RadiansPerSecond), 1e-9);

    Setpoint<AngularVelocityUnit> goal =
        new Setpoint<>(RadiansPerSecond.of(20), RadiansPerSecond.of(1));
    mechanism.setVelocityGoal(goal);
    assertSame(goal, mechanism.getVelocityGoal());

    mechanism.setVelocityGoal(RadiansPerSecond.of(30), Amps.of(2));
    assertEquals("VelocityCurrent", mechanism.getMode());
    assertEquals(Amps.of(2), mechanism.getVelocityFeedforward());

    Setpoint<AngularVelocityUnit> other =
        new Setpoint<>(RadiansPerSecond.of(10), RadiansPerSecond.of(1));
    mechanism.setVelocityGoal(other, Amps.of(1));
    assertSame(other, mechanism.getVelocityGoal());

    motors.read(t -> t.velocity = RadiansPerSecond.of(10));
    assertTrue(mechanism.atVelocityGoal());
    assertFalse(mechanism.atVelocityGoal(RadiansPerSecond.of(40)));

    Command wait = mechanism.waitUntilAtVelocityGoal();
    wait.initialize();
    assertFalse(wait.isFinished());
  }
}
