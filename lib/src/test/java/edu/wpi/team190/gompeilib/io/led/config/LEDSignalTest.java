package edu.wpi.team190.gompeilib.io.led.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class LEDSignalTest {

  @Test
  public void testDefaultRatesAreEachGroupsDefault() {
    for (LEDSignalGroup group : LEDSignalGroup.values()) {
      assertEquals(group.defaultRate(), LEDSignal.DEFAULT_RATES.rateOf(group));
    }
    assertTrue(LEDSignal.DEFAULT_RATES.signalOverrides().isEmpty());
  }

  @Test
  public void testSignalsUseTheirGroupsRate() {
    for (LEDSignal signal : LEDSignal.values()) {
      assertEquals(signal.group().defaultRate(), LEDSignal.DEFAULT_RATES.rateFor(signal));
    }
    assertEquals(Hertz.of(4.0), LEDSignal.DEFAULT_RATES.rateFor(LEDSignal.SUPPLY_VOLTAGE));
  }
}
