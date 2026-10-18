package edu.wpi.team190.gompeilib.io.motor.config;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import java.util.Objects;
import java.util.Optional;

/**
 * A motor that follows a leader. Everything but the CAN ID, bus, and alignment comes from the
 * leader's config.
 *
 * @param canId the follower's CAN ID
 * @param alignment whether the follower's output is aligned with or opposed to the leader's
 * @param bus the follower's bus, or empty for the leader's bus
 * @param factory how to build the follower, or empty to use the leader's factory
 */
public record FollowerMotor(
    int canId, MotorAlignment alignment, Optional<CANBusId> bus, Optional<MotorIOFactory> factory) {
  public FollowerMotor {
    Objects.requireNonNull(alignment, "alignment");
    Objects.requireNonNull(bus, "bus");
    Objects.requireNonNull(factory, "factory");
  }

  /**
   * Creates a follower on the leader's bus, built by the leader's factory.
   *
   * @param canId the follower's CAN ID
   * @param alignment the follower's alignment to the leader
   */
  public FollowerMotor(int canId, MotorAlignment alignment) {
    this(canId, alignment, Optional.empty(), Optional.empty());
  }

  /**
   * Returns this follower on another bus.
   *
   * @param bus the follower's bus
   * @return the updated follower
   */
  public FollowerMotor onBus(CANBusId bus) {
    return new FollowerMotor(canId, alignment, Optional.of(bus), factory);
  }

  /**
   * Returns this follower built by another factory, e.g. a different vendor.
   *
   * @param factory the follower's factory
   * @return the updated follower
   */
  public FollowerMotor builtBy(MotorIOFactory factory) {
    return new FollowerMotor(canId, alignment, bus, Optional.of(factory));
  }
}
