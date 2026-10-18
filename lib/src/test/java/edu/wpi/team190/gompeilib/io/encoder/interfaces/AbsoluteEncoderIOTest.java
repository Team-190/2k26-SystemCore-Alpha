package edu.wpi.team190.gompeilib.io.encoder.interfaces;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Rotations;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetryAutoLogged;
import org.junit.jupiter.api.Test;

public class AbsoluteEncoderIOTest {

  @Test
  public void testDefaultMethods() throws IllegalAccessException {
    AbsoluteEncoderIO io = new AbsoluteEncoderIO() {};
    AbsoluteEncoderTelemetryAutoLogged telemetry = new AbsoluteEncoderTelemetryAutoLogged();

    io.updateInputs(telemetry);
    io.setPosition(Rotations.of(1.0));
    io.clearStickyFaults();

    assertFalse(io.isConnected());
    IOTestSupport.assertZeroDefaults(telemetry);
  }
}
