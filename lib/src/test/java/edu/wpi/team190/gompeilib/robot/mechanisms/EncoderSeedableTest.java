package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.mockito.Mockito.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMechanism;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import org.junit.jupiter.api.Test;

public class EncoderSeedableTest extends GompeiLibTestBase {
  @Test
  public void testSeedsThroughTheEncoderSeed() {
    TestMotors motors = new TestMotors();
    TestMechanism mechanism = new TestMechanism(motors);

    // The test mechanism has no encoder, so seeding leaves the motors alone
    mechanism.seedPositionFromEncoder();
    verify(motors.motor, never()).setPosition(any());
  }
}
