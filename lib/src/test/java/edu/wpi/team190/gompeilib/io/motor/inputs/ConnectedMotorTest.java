package edu.wpi.team190.gompeilib.io.motor.inputs;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class ConnectedMotorTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(
        ConnectedMotor.class,
        "UNKNOWN",
        "FALCON_500",
        "KRAKEN_X60",
        "KRAKEN_X44",
        "MINION",
        "BRUSHED_AB",
        "BRUSHED_AC",
        "BRUSHED_BC",
        "NEO",
        "NEO_550",
        "VORTEX",
        "CUSTOM_BRUSHLESS");
  }
}
