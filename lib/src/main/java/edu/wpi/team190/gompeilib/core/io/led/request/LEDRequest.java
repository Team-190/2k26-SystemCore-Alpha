package edu.wpi.team190.gompeilib.core.io.led.request;

import edu.wpi.team190.gompeilib.core.io.led.LEDColor;
import edu.wpi.team190.gompeilib.core.io.led.LEDRange;
import org.wpilib.units.measure.Frequency;

/**
 * A vendor-neutral LED controller request. There is exactly one record per vendor control request,
 * so a vendor implementation is an exhaustive {@code switch} over this hierarchy.
 */
public sealed interface LEDRequest
    permits LEDRequest.Solid, LEDRequest.SupplyModulation, LEDAnimation {

  /**
   * Sets a range of LEDs to one color.
   *
   * @param range the LEDs to set
   * @param color the color
   */
  record Solid(LEDRange range, LEDColor color) implements LEDRequest {}

  /**
   * Modulates the controller's supply-voltage output, e.g. to drive a single-color strip.
   *
   * @param output the modulation duty cycle, in [0, 1]
   * @param updateFrequency how often the request is re-sent
   */
  record SupplyModulation(double output, Frequency updateFrequency) implements LEDRequest {}
}
