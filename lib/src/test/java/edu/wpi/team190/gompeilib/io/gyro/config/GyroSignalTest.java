package edu.wpi.team190.gompeilib.io.gyro.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class GyroSignalTest {

  @Test
  public void testDefaultRatesAreEachGroupsDefault() {
    for (GyroSignalGroup group : GyroSignalGroup.values()) {
      assertEquals(group.defaultRate(), GyroSignal.DEFAULT_RATES.rateOf(group));
    }
    assertTrue(GyroSignal.DEFAULT_RATES.signalOverrides().isEmpty());
  }

  @Test
  public void testSignalsUseTheirGroupsRate() {
    for (GyroSignal signal : GyroSignal.values()) {
      assertEquals(signal.group().defaultRate(), GyroSignal.DEFAULT_RATES.rateFor(signal));
    }
    assertEquals(Hertz.of(100.0), GyroSignal.DEFAULT_RATES.rateFor(GyroSignal.YAW_POSITION));
  }
}
