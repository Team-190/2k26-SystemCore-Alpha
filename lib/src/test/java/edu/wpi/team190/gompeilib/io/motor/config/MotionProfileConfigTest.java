package edu.wpi.team190.gompeilib.io.motor.config;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import org.junit.jupiter.api.Test;

public class MotionProfileConfigTest {

  @Test
  public void testNoneIsAllZero() {
    MotionProfileConfig none = MotionProfileConfig.NONE;

    assertEquals(0.0, none.cruiseVelocity().in(RotationsPerSecond));
    assertEquals(0.0, none.acceleration().in(RotationsPerSecondPerSecond));
    assertEquals(0.0, none.jerk().in(RotationsPerSecondPerSecond.per(Second)));
    assertEquals(0.0, none.exponentialKV().in(Volts.per(RotationsPerSecond)));
    assertEquals(0.0, none.exponentialKA().in(Volts.per(RotationsPerSecondPerSecond)));
  }

  @Test
  public void testComponents() {
    MotionProfileConfig profile =
        new MotionProfileConfig(
            RotationsPerSecond.of(1.0),
            RotationsPerSecondPerSecond.of(2.0),
            RotationsPerSecondPerSecond.per(Second).of(3.0),
            Volts.per(RotationsPerSecond).ofNative(0.12),
            Volts.per(RotationsPerSecondPerSecond).ofNative(0.01));

    assertEquals(1.0, profile.cruiseVelocity().in(RotationsPerSecond), 1e-9);
    assertEquals(2.0, profile.acceleration().in(RotationsPerSecondPerSecond), 1e-9);
    assertEquals(3.0, profile.jerk().in(RotationsPerSecondPerSecond.per(Second)), 1e-9);
    assertEquals(0.12, profile.exponentialKV().in(Volts.per(RotationsPerSecond)), 1e-9);
    assertEquals(0.01, profile.exponentialKA().in(Volts.per(RotationsPerSecondPerSecond)), 1e-9);
  }
}
