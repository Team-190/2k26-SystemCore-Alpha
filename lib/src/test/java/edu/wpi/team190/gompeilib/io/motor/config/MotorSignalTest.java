package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class MotorSignalTest {

  @Test
  public void testDefaultRatesAreEachGroupsDefault() {
    for (MotorSignalGroup group : MotorSignalGroup.values()) {
      assertEquals(group.defaultRate(), MotorSignal.DEFAULT_RATES.rateOf(group));
    }
    assertTrue(MotorSignal.DEFAULT_RATES.signalOverrides().isEmpty());
  }

  @Test
  public void testSignalsUseTheirGroupsRate() {
    for (MotorSignal signal : MotorSignal.values()) {
      assertEquals(signal.group().defaultRate(), MotorSignal.DEFAULT_RATES.rateFor(signal));
    }
    assertEquals(Hertz.of(100.0), MotorSignal.DEFAULT_RATES.rateFor(MotorSignal.POSITION));
  }
}
