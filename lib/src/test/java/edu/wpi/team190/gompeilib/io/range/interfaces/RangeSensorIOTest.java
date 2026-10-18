package edu.wpi.team190.gompeilib.io.range.interfaces;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.range.inputs.RangeSensorTelemetryAutoLogged;
import org.junit.jupiter.api.Test;

public class RangeSensorIOTest {

  @Test
  public void testDefaultMethods() throws IllegalAccessException {
    RangeSensorIO io = new RangeSensorIO() {};
    RangeSensorTelemetryAutoLogged telemetry = new RangeSensorTelemetryAutoLogged();

    io.updateInputs(telemetry);
    io.clearStickyFaults();

    assertFalse(io.isConnected());
    IOTestSupport.assertZeroDefaults(telemetry);
  }
}
