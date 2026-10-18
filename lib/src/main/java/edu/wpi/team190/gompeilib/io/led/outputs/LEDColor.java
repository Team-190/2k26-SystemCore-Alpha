package edu.wpi.team190.gompeilib.io.led.outputs;

import org.wpilib.util.Color;

/**
 * An RGBW LED color, each channel in [0, 255].
 *
 * @param red the red channel
 * @param green the green channel
 * @param blue the blue channel
 * @param white the white channel, ignored by RGB-only strips
 */
public record LEDColor(int red, int green, int blue, int white) {
  public LEDColor {
    checkChannel("red", red);
    checkChannel("green", green);
    checkChannel("blue", blue);
    checkChannel("white", white);
  }

  /**
   * Creates an RGB color with the white channel off.
   *
   * @param red the red channel
   * @param green the green channel
   * @param blue the blue channel
   */
  public LEDColor(int red, int green, int blue) {
    this(red, green, blue, 0);
  }

  /**
   * Converts a WPILib color to an LED color with the white channel off.
   *
   * @param color the WPILib color, each channel in [0, 1]
   * @return the equivalent LED color
   */
  public static LEDColor from(Color color) {
    return new LEDColor(toChannel(color.red), toChannel(color.green), toChannel(color.blue));
  }

  private static int toChannel(double value) {
    return (int) Math.round(Math.clamp(value, 0.0, 1.0) * 255.0);
  }

  private static void checkChannel(String name, int value) {
    if (value < 0 || value > 255) {
      throw new IllegalArgumentException(name + " must be in [0, 255], got " + value);
    }
  }
}
