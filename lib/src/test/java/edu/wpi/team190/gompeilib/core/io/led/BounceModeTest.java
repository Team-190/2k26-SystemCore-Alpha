package edu.wpi.team190.gompeilib.core.io.led;

import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class BounceModeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(BounceMode.class, "FRONT", "CENTER", "BACK");
  }
}
