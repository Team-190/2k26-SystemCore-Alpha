package edu.wpi.team190.gompeilib.io.encoder.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class AbsoluteEncoderSignalTest {

  @Test
  public void testDefaultRatesAreEachGroupsDefault() {
    for (AbsoluteEncoderSignalGroup group : AbsoluteEncoderSignalGroup.values()) {
      assertEquals(group.defaultRate(), AbsoluteEncoderSignal.DEFAULT_RATES.rateOf(group));
    }
    assertTrue(AbsoluteEncoderSignal.DEFAULT_RATES.signalOverrides().isEmpty());
  }

  @Test
  public void testSignalsUseTheirGroupsRate() {
    for (AbsoluteEncoderSignal signal : AbsoluteEncoderSignal.values()) {
      assertEquals(
          signal.group().defaultRate(), AbsoluteEncoderSignal.DEFAULT_RATES.rateFor(signal));
    }
    assertEquals(
        Hertz.of(100.0),
        AbsoluteEncoderSignal.DEFAULT_RATES.rateFor(AbsoluteEncoderSignal.POSITION));
  }
}
