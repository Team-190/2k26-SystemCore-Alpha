package edu.wpi.team190.gompeilib.io.range.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class RangeSensorSignalTest {

  @Test
  public void testDefaultRatesAreEachGroupsDefault() {
    for (RangeSensorSignalGroup group : RangeSensorSignalGroup.values()) {
      assertEquals(group.defaultRate(), RangeSensorSignal.DEFAULT_RATES.rateOf(group));
    }
    assertTrue(RangeSensorSignal.DEFAULT_RATES.signalOverrides().isEmpty());
  }

  @Test
  public void testSignalsUseTheirGroupsRate() {
    for (RangeSensorSignal signal : RangeSensorSignal.values()) {
      assertEquals(signal.group().defaultRate(), RangeSensorSignal.DEFAULT_RATES.rateFor(signal));
    }
    assertEquals(
        Hertz.of(100.0), RangeSensorSignal.DEFAULT_RATES.rateFor(RangeSensorSignal.DISTANCE));
  }
}
