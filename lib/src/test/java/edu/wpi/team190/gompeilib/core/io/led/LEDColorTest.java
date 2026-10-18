package edu.wpi.team190.gompeilib.core.io.led;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.wpilib.util.Color;

public class LEDColorTest {

  @Test
  public void testAccessors() {
    LEDColor color = new LEDColor(1, 2, 3, 4);

    assertEquals(1, color.red());
    assertEquals(2, color.green());
    assertEquals(3, color.blue());
    assertEquals(4, color.white());
  }

  @Test
  public void testRgbConstructorTurnsWhiteOff() {
    assertEquals(new LEDColor(10, 20, 30, 0), new LEDColor(10, 20, 30));
  }

  @Test
  public void testBoundaryChannels() {
    assertDoesNotThrow(() -> new LEDColor(0, 0, 0, 0));
    assertDoesNotThrow(() -> new LEDColor(255, 255, 255, 255));
  }

  @Test
  public void shouldRejectOutOfRangeChannels() {
    assertThrows(IllegalArgumentException.class, () -> new LEDColor(-1, 0, 0, 0));
    assertThrows(IllegalArgumentException.class, () -> new LEDColor(0, 256, 0, 0));
    assertThrows(IllegalArgumentException.class, () -> new LEDColor(0, 0, 300, 0));
    assertThrows(IllegalArgumentException.class, () -> new LEDColor(0, 0, 0, -5));
  }

  @Test
  public void testFromWpilibColor() {
    assertEquals(new LEDColor(255, 0, 0), LEDColor.from(new Color(1.0, 0.0, 0.0)));
    assertEquals(new LEDColor(0, 128, 255), LEDColor.from(new Color(0.0, 0.5, 1.0)));
  }

  @Test
  public void testFromClampsOutOfRangeChannels() {
    assertEquals(new LEDColor(255, 0, 0), LEDColor.from(new Color(1.5, -0.5, 0.0)));
  }
}
