package edu.wpi.team190.gompeilib.io.motor.config;

import java.util.Objects;
import java.util.Optional;
import org.wpilib.units.measure.Angle;

/**
 * Positions, at the gearbox output, past which the motor refuses to drive further. An empty limit
 * is disabled.
 *
 * @param forward the forward limit
 * @param reverse the reverse limit
 */
public record SoftLimits(Optional<Angle> forward, Optional<Angle> reverse) {
  /** No soft limits. */
  public static final SoftLimits NONE = new SoftLimits(Optional.empty(), Optional.empty());

  public SoftLimits {
    Objects.requireNonNull(forward, "forward");
    Objects.requireNonNull(reverse, "reverse");
  }

  /**
   * Returns limits in both directions.
   *
   * @param reverse the reverse limit
   * @param forward the forward limit
   * @return the limits
   */
  public static SoftLimits between(Angle reverse, Angle forward) {
    return new SoftLimits(Optional.of(forward), Optional.of(reverse));
  }
}
