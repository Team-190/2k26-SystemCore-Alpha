package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Amps;

import org.junit.jupiter.api.Test;

public class TorqueCurrentLimitsTest {

  @Test
  public void testNoneLeavesTheHardwareLimits() {
    assertEquals(Amps.of(800.0), TorqueCurrentLimits.NONE.peakForward());
    assertEquals(Amps.of(-800.0), TorqueCurrentLimits.NONE.peakReverse());
  }

  @Test
  public void testSymmetric() {
    TorqueCurrentLimits limits = TorqueCurrentLimits.symmetric(Amps.of(40.0));

    assertEquals(Amps.of(40.0), limits.peakForward());
    assertEquals(Amps.of(-40.0), limits.peakReverse());
  }

  @Test
  public void shouldRejectNulls() {
    assertThrows(NullPointerException.class, () -> new TorqueCurrentLimits(null, Amps.of(0.0)));
    assertThrows(NullPointerException.class, () -> new TorqueCurrentLimits(Amps.of(0.0), null));
  }
}
