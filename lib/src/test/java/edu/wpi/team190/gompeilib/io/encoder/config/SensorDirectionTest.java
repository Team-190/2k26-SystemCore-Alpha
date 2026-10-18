package edu.wpi.team190.gompeilib.io.encoder.config;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class SensorDirectionTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(
        SensorDirection.class, "COUNTER_CLOCKWISE_POSITIVE", "CLOCKWISE_POSITIVE");
  }
}
