package edu.wpi.team190.gompeilib.core.io.range;

import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class MeasurementHealthTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(MeasurementHealth.class, "UNKNOWN", "GOOD", "LIMITED", "BAD");
  }
}
