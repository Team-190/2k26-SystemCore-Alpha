package edu.wpi.team190.gompeilib.io.motor.interfaces;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import org.junit.jupiter.api.Test;

public class MotorIOFactoryTest {

  @Test
  public void testHardwareFollowingDefaultsOff() {
    MotorIOFactory factory =
        new MotorIOFactory() {
          @Override
          public MotorIO create(MotorConfig config) {
            return MotorIO.replay();
          }

          @Override
          public String family() {
            return "Test";
          }
        };

    assertFalse(factory.supportsHardwareFollowing());
    assertEquals("Test", factory.family());
    assertNotNull(factory.create(null));
  }
}
