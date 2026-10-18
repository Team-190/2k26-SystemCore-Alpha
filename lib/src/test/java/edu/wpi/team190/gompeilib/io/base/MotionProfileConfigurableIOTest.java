package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import org.junit.jupiter.api.Test;

public class MotionProfileConfigurableIOTest {

  @Test
  public void testDefaultMethods() {
    MotionProfileConfigurableIO io = new MotionProfileConfigurableIO() {};

    assertDoesNotThrow(() -> io.updateMotionProfile(MotionProfileConfig.NONE));
  }
}
