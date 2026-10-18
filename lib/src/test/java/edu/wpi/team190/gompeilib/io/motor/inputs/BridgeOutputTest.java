package edu.wpi.team190.gompeilib.io.motor.inputs;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class BridgeOutputTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(
        BridgeOutput.class,
        "UNKNOWN",
        "COAST",
        "BRAKE",
        "TRAPEZOIDAL",
        "FOC_TORQUE",
        "TONE",
        "FOC_EASY",
        "FAULT_BRAKE",
        "FAULT_COAST",
        "ACTIVE_BRAKE",
        "VARIABLE_BRAKE");
  }
}
