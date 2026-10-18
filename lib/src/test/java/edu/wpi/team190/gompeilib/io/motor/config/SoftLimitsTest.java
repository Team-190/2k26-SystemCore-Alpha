package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Rotations;

import java.util.Optional;
import org.junit.jupiter.api.Test;

public class SoftLimitsTest {

  @Test
  public void testNoneIsDisabled() {
    assertTrue(SoftLimits.NONE.forward().isEmpty());
    assertTrue(SoftLimits.NONE.reverse().isEmpty());
  }

  @Test
  public void testBetween() {
    SoftLimits limits = SoftLimits.between(Rotations.of(-1.0), Rotations.of(2.0));

    assertEquals(Optional.of(Rotations.of(2.0)), limits.forward());
    assertEquals(Optional.of(Rotations.of(-1.0)), limits.reverse());
  }

  @Test
  public void shouldRejectNulls() {
    assertThrows(NullPointerException.class, () -> new SoftLimits(null, Optional.empty()));
    assertThrows(NullPointerException.class, () -> new SoftLimits(Optional.empty(), null));
  }
}
