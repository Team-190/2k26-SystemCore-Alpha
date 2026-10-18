package edu.wpi.team190.gompeilib.io.range.config;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class RangeUpdateModeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(
        RangeUpdateMode.class,
        "SHORT_RANGE_100HZ",
        "SHORT_RANGE_USER_FREQUENCY",
        "LONG_RANGE_USER_FREQUENCY");
  }
}
