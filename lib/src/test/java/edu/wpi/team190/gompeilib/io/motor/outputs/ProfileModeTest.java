package edu.wpi.team190.gompeilib.io.motor.outputs;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class ProfileModeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(ProfileMode.class, "PROFILED", "UNPROFILED", "EXPONENTIAL");
  }
}
