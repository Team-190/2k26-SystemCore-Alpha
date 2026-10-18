package edu.wpi.team190.gompeilib.io.led.interfaces;

import edu.wpi.team190.gompeilib.io.base.ConnectedIO;
import edu.wpi.team190.gompeilib.io.base.RequestControllableIO;
import edu.wpi.team190.gompeilib.io.base.StickyFaultClearableIO;
import edu.wpi.team190.gompeilib.io.led.inputs.LEDTelemetry;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRequest;

/**
 * One LED controller. Implementations run every {@link LEDRequest} the device supports and throw
 * {@link UnsupportedOperationException} for the rest.
 */
public interface LEDIO
    extends ConnectedIO, RequestControllableIO<LEDRequest>, StickyFaultClearableIO {
  /**
   * Fills {@code telemetry} with the controller's latest readings.
   *
   * @param telemetry the telemetry to fill
   */
  default void updateInputs(LEDTelemetry telemetry) {}
}
