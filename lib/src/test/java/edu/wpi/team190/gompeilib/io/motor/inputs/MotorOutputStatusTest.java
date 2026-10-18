package edu.wpi.team190.gompeilib.io.motor.inputs;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class MotorOutputStatusTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(
        MotorOutputStatus.class,
        "UNKNOWN",
        "OFF",
        "STATIC_BRAKING",
        "MOTORING",
        "DISCORDANT_MOTORING",
        "REGEN_BRAKING");
  }
}
