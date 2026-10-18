package edu.wpi.team190.gompeilib.io.gyro.interfaces;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Degrees;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetryAutoLogged;
import org.junit.jupiter.api.Test;

public class GyroIOTest {

  @Test
  public void testDefaultMethods() throws IllegalAccessException {
    GyroIO io = new GyroIO() {};
    GyroTelemetryAutoLogged telemetry = new GyroTelemetryAutoLogged();

    io.updateInputs(telemetry);
    io.setYaw(Degrees.of(90.0));
    io.clearStickyFaults();

    assertFalse(io.isConnected());
    IOTestSupport.assertZeroDefaults(telemetry);
  }
}
