package edu.wpi.team190.gompeilib.core.io.motor.request;

import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class OutputTypeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(OutputType.class, "DUTY_CYCLE", "VOLTAGE", "CURRENT");
  }
}
