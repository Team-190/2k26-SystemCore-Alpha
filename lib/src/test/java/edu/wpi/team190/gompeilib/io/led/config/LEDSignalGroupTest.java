package edu.wpi.team190.gompeilib.io.led.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class LEDSignalGroupTest {

  @Test
  public void testDefaultRatesAreNonNegative() {
    for (LEDSignalGroup group : LEDSignalGroup.values()) {
      assertTrue(group.defaultRate().in(Hertz) >= 0.0, group.toString());
    }
  }
}
