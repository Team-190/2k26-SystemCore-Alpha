package edu.wpi.team190.gompeilib.io.encoder.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class AbsoluteEncoderSignalGroupTest {

  @Test
  public void testDefaultRatesAreNonNegative() {
    for (AbsoluteEncoderSignalGroup group : AbsoluteEncoderSignalGroup.values()) {
      assertTrue(group.defaultRate().in(Hertz) >= 0.0, group.toString());
    }
  }
}
