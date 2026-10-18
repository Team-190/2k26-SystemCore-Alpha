package edu.wpi.team190.gompeilib.io.led.config;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class LEDStripTypeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(
        LEDStripType.class, "GRB", "RGB", "BRG", "GRBW", "RGBW", "BRGW");
  }
}
