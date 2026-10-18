package edu.wpi.team190.gompeilib.io.range.interfaces;

import edu.wpi.team190.gompeilib.io.base.ConnectedIO;
import edu.wpi.team190.gompeilib.io.base.StickyFaultClearableIO;
import edu.wpi.team190.gompeilib.io.range.inputs.RangeSensorTelemetry;

/** One range (time-of-flight) sensor. */
public interface RangeSensorIO extends ConnectedIO, StickyFaultClearableIO {
  /**
   * Fills {@code telemetry} with the sensor's latest readings.
   *
   * @param telemetry the telemetry to fill
   */
  default void updateInputs(RangeSensorTelemetry telemetry) {}
}
