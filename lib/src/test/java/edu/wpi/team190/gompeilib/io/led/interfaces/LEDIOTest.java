package edu.wpi.team190.gompeilib.io.led.interfaces;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.led.inputs.LEDTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDColor;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRange;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRequest;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class LEDIOTest {

  @Test
  public void testDefaultMethods() throws IllegalAccessException {
    List<LEDRequest> sent = new ArrayList<>();
    LEDIO io = sent::add;
    LEDTelemetryAutoLogged telemetry = new LEDTelemetryAutoLogged();

    io.updateInputs(telemetry);
    io.clearStickyFaults();
    io.setControl(new LEDRequest.Solid(new LEDRange(0, 3), new LEDColor(255, 0, 0)));

    assertFalse(io.isConnected());
    assertEquals(1, sent.size());
    IOTestSupport.assertZeroDefaults(telemetry);
  }
}
