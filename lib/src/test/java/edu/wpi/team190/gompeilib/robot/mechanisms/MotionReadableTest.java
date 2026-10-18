package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class MotionReadableTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  @Test
  public void testReadsTheLeaderThroughTheConversion() {
    TestMotors motors = new TestMotors();
    TestMechanism mechanism = new TestMechanism(motors);
    motors.read(
        t -> {
          t.position = Degrees.of(45);
          t.velocity = RadiansPerSecond.of(3);
          t.closedLoopPositionReferenceSlope = RadiansPerSecond.of(2);
        });

    assertEquals(45.0, mechanism.getPosition().in(Degrees), 1e-9);
    assertEquals(3.0, mechanism.getVelocity().in(RadiansPerSecond), 1e-9);
    assertEquals(2.0, mechanism.getVelocitySetpoint().in(RadiansPerSecond), 1e-9);
  }
}
