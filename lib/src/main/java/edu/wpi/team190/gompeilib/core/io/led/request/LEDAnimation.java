package edu.wpi.team190.gompeilib.core.io.led.request;

import edu.wpi.team190.gompeilib.core.io.led.AnimationDirection;
import edu.wpi.team190.gompeilib.core.io.led.BounceMode;
import edu.wpi.team190.gompeilib.core.io.led.LEDColor;
import edu.wpi.team190.gompeilib.core.io.led.LEDRange;
import org.wpilib.units.measure.Frequency;

/** An LED animation that runs in one of the controller's animation slots. */
public sealed interface LEDAnimation extends LEDRequest {
  /**
   * Returns the animation slot this request occupies.
   *
   * @return the animation slot
   */
  int slot();

  /**
   * Returns how often the request is re-sent.
   *
   * @return the update frequency
   */
  Frequency updateFrequency();

  /**
   * A block of color that flows along the strip.
   *
   * @param range the LEDs the animation covers
   * @param slot the animation slot
   * @param color the color
   * @param direction the direction of travel
   * @param frameRate how fast the animation advances
   * @param updateFrequency how often the request is re-sent
   */
  record ColorFlow(
      LEDRange range,
      int slot,
      LEDColor color,
      AnimationDirection direction,
      Frequency frameRate,
      Frequency updateFrequency)
      implements LEDAnimation {}

  /**
   * A flickering fire effect.
   *
   * @param range the LEDs the animation covers
   * @param slot the animation slot
   * @param brightness the brightness, in [0, 1]
   * @param direction the direction the flames rise
   * @param sparking how often new sparks appear, in [0, 1]
   * @param cooling how quickly sparks fade, in [0, 1]
   * @param frameRate how fast the animation advances
   * @param updateFrequency how often the request is re-sent
   */
  record Fire(
      LEDRange range,
      int slot,
      double brightness,
      AnimationDirection direction,
      double sparking,
      double cooling,
      Frequency frameRate,
      Frequency updateFrequency)
      implements LEDAnimation {}

  /**
   * A pocket of light that scans back and forth.
   *
   * @param range the LEDs the animation covers
   * @param slot the animation slot
   * @param color the color
   * @param size the pocket's size, in LEDs
   * @param bounce where the pocket turns around
   * @param frameRate how fast the animation advances
   * @param updateFrequency how often the request is re-sent
   */
  record Larson(
      LEDRange range,
      int slot,
      LEDColor color,
      int size,
      BounceMode bounce,
      Frequency frameRate,
      Frequency updateFrequency)
      implements LEDAnimation {}

  /**
   * A moving rainbow.
   *
   * @param range the LEDs the animation covers
   * @param slot the animation slot
   * @param brightness the brightness, in [0, 1]
   * @param direction the direction of travel
   * @param frameRate how fast the animation advances
   * @param updateFrequency how often the request is re-sent
   */
  record Rainbow(
      LEDRange range,
      int slot,
      double brightness,
      AnimationDirection direction,
      Frequency frameRate,
      Frequency updateFrequency)
      implements LEDAnimation {}

  /**
   * Fades through red, green, and blue.
   *
   * @param range the LEDs the animation covers
   * @param slot the animation slot
   * @param brightness the brightness, in [0, 1]
   * @param frameRate how fast the animation advances
   * @param updateFrequency how often the request is re-sent
   */
  record RgbFade(
      LEDRange range, int slot, double brightness, Frequency frameRate, Frequency updateFrequency)
      implements LEDAnimation {}

  /**
   * Fades one color in and out.
   *
   * @param range the LEDs the animation covers
   * @param slot the animation slot
   * @param color the color
   * @param frameRate how fast the animation advances
   * @param updateFrequency how often the request is re-sent
   */
  record SingleFade(
      LEDRange range, int slot, LEDColor color, Frequency frameRate, Frequency updateFrequency)
      implements LEDAnimation {}

  /**
   * Flashes one color on and off.
   *
   * @param range the LEDs the animation covers
   * @param slot the animation slot
   * @param color the color
   * @param frameRate how fast the animation advances
   * @param updateFrequency how often the request is re-sent
   */
  record Strobe(
      LEDRange range, int slot, LEDColor color, Frequency frameRate, Frequency updateFrequency)
      implements LEDAnimation {}

  /**
   * Random LEDs twinkle on in one color.
   *
   * @param range the LEDs the animation covers
   * @param slot the animation slot
   * @param color the color
   * @param maxOnProportion the largest share of LEDs lit at once, in [0, 1]
   * @param frameRate how fast the animation advances
   * @param updateFrequency how often the request is re-sent
   */
  record Twinkle(
      LEDRange range,
      int slot,
      LEDColor color,
      double maxOnProportion,
      Frequency frameRate,
      Frequency updateFrequency)
      implements LEDAnimation {}

  /**
   * Random LEDs twinkle off from one color.
   *
   * @param range the LEDs the animation covers
   * @param slot the animation slot
   * @param color the color
   * @param maxOnProportion the largest share of LEDs lit at once, in [0, 1]
   * @param frameRate how fast the animation advances
   * @param updateFrequency how often the request is re-sent
   */
  record TwinkleOff(
      LEDRange range,
      int slot,
      LEDColor color,
      double maxOnProportion,
      Frequency frameRate,
      Frequency updateFrequency)
      implements LEDAnimation {}

  /**
   * Stops whatever animation is running in a slot.
   *
   * @param slot the animation slot to clear
   * @param updateFrequency how often the request is re-sent
   */
  record ClearAnimation(int slot, Frequency updateFrequency) implements LEDAnimation {}
}
