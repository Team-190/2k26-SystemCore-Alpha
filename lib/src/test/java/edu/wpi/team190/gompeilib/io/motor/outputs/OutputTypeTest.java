package edu.wpi.team190.gompeilib.io.motor.outputs;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class OutputTypeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(OutputType.class, "DUTY_CYCLE", "VOLTAGE", "CURRENT");
  }
}
