package edu.wpi.team190.gompeilib.core.io.led;

import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class AnimationDirectionTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(AnimationDirection.class, "FORWARD", "BACKWARD");
  }
}
