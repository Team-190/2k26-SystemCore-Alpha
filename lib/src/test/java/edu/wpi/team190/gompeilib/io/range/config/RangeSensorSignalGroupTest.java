package edu.wpi.team190.gompeilib.io.range.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class RangeSensorSignalGroupTest {

  @Test
  public void testDefaultRatesAreNonNegative() {
    for (RangeSensorSignalGroup group : RangeSensorSignalGroup.values()) {
      assertTrue(group.defaultRate().in(Hertz) >= 0.0, group.toString());
    }
  }
}
