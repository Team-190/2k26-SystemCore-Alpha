package edu.wpi.team190.gompeilib.io.range.inputs;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class MeasurementHealthTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(MeasurementHealth.class, "UNKNOWN", "GOOD", "LIMITED", "BAD");
  }
}
