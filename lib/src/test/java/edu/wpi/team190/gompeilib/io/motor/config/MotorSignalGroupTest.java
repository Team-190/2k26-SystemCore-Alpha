package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class MotorSignalGroupTest {

  @Test
  public void testDefaultRatesAreNonNegative() {
    for (MotorSignalGroup group : MotorSignalGroup.values()) {
      assertTrue(group.defaultRate().in(Hertz) >= 0.0, group.toString());
    }
  }

  @Test
  public void testDefaultsLetHardwareFollowersFollow() {
    assertTrue(MotorSignalGroup.OUTPUT.defaultRate().gte(MotorSignalGroup.MIN_FOLLOWED_RATE));
    assertEquals(Hertz.of(20.0), MotorSignalGroup.MIN_FOLLOWED_RATE);
  }

  @Test
  public void testDifferentialIsOffByDefault() {
    assertEquals(0.0, MotorSignalGroup.DIFFERENTIAL.defaultRate().in(Hertz));
  }
}
