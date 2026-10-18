package edu.wpi.team190.gompeilib.io.led.outputs;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class BounceModeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(BounceMode.class, "FRONT", "CENTER", "BACK");
  }
}
