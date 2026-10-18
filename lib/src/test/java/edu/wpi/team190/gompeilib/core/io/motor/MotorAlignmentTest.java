package edu.wpi.team190.gompeilib.core.io.motor;

import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class MotorAlignmentTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(MotorAlignment.class, "ALIGNED", "OPPOSED");
  }
}
