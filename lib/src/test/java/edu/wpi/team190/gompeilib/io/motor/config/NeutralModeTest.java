package edu.wpi.team190.gompeilib.io.motor.config;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class NeutralModeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(NeutralMode.class, "COAST", "BRAKE");
  }
}
