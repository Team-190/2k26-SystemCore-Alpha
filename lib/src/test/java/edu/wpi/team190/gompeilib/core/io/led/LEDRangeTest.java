package edu.wpi.team190.gompeilib.core.io.led;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class LEDRangeTest {

  @Test
  public void testAccessorsAndLength() {
    LEDRange range = new LEDRange(8, 15);

    assertEquals(8, range.start());
    assertEquals(15, range.end());
    assertEquals(8, range.length());
  }

  @Test
  public void testSingleLED() {
    assertEquals(1, new LEDRange(0, 0).length());
  }

  @Test
  public void shouldRejectNegativeStart() {
    assertThrows(IllegalArgumentException.class, () -> new LEDRange(-1, 4));
  }

  @Test
  public void shouldRejectEndBeforeStart() {
    assertThrows(IllegalArgumentException.class, () -> new LEDRange(5, 4));
  }
}
