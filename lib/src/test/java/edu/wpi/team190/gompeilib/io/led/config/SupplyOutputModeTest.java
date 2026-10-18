package edu.wpi.team190.gompeilib.io.led.config;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class SupplyOutputModeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(SupplyOutputMode.class, "ON", "OFF", "MODULATED");
  }
}
