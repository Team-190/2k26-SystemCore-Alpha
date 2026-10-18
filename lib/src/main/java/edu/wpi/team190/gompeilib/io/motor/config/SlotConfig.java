package edu.wpi.team190.gompeilib.io.motor.config;

import edu.wpi.team190.gompeilib.control.Gains;
import java.util.Objects;

/**
 * The closed-loop gains in one slot. Gains are in output units (duty cycle, volts, or amps,
 * matching the request's output type) per rotation, rotation per second, or rotation per second
 * squared at the gearbox output.
 *
 * @param gains the feedback and feedforward gains
 * @param gravityType how {@code kG} is applied
 */
public record SlotConfig(Gains gains, GravityType gravityType) {
  public SlotConfig {
    Objects.requireNonNull(gains, "gains");
    Objects.requireNonNull(gravityType, "gravityType");
  }

  /**
   * Creates a slot whose {@code kG} is a constant.
   *
   * @param gains the gains
   */
  public SlotConfig(Gains gains) {
    this(gains, GravityType.CONSTANT);
  }
}
