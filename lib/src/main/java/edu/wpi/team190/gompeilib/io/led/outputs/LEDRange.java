package edu.wpi.team190.gompeilib.io.led.outputs;

/**
 * An inclusive range of LED indices on a strip.
 *
 * @param start the first LED index, at least zero
 * @param end the last LED index, at least {@code start}
 */
public record LEDRange(int start, int end) {
  public LEDRange {
    if (start < 0) {
      throw new IllegalArgumentException("start must be at least 0, got " + start);
    }
    if (end < start) {
      throw new IllegalArgumentException("end must be at least start (" + start + "), got " + end);
    }
  }

  /**
   * Returns the number of LEDs in the range.
   *
   * @return {@code end - start + 1}
   */
  public int length() {
    return end - start + 1;
  }
}
