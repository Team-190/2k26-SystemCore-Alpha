package edu.wpi.team190.gompeilib.io.motor.inputs;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class RotorPolarityTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(
        RotorPolarity.class, "UNKNOWN", "COUNTER_CLOCKWISE_POSITIVE", "CLOCKWISE_POSITIVE");
  }
}
