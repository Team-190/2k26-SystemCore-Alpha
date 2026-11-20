package edu.wpi.team190.gompeilib.io.gyro.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class GyroSignalGroupTest {

  @Test
  public void testDefaultRatesAreNonNegative() {
    for (GyroSignalGroup group : GyroSignalGroup.values()) {
      assertTrue(group.defaultRate().in(Hertz) >= 0.0, group.toString());
    }
  }
}
