package edu.wpi.team190.gompeilib.io.encoder.interfaces;

import edu.wpi.team190.gompeilib.io.base.ConnectedIO;
import edu.wpi.team190.gompeilib.io.base.PositionResettableIO;
import edu.wpi.team190.gompeilib.io.base.StickyFaultClearableIO;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry;

/** One absolute encoder. */
public interface AbsoluteEncoderIO
    extends ConnectedIO, PositionResettableIO, StickyFaultClearableIO {
  /**
   * Fills {@code telemetry} with the encoder's latest readings.
   *
   * @param telemetry the telemetry to fill
   */
  default void updateInputs(AbsoluteEncoderTelemetry telemetry) {}
}
