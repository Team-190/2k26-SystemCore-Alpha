package edu.wpi.team190.gompeilib.io.motor.config;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import java.util.Objects;
import java.util.Optional;

/**
 * A motor that follows a leader. Everything but the CAN ID, bus, alignment, and follow type comes
 * from the leader's config.
 *
 * @param canId the follower's CAN ID
 * @param alignment whether the follower's output is aligned with or opposed to the leader's
 * @param followType the type of follower tracking (standard, strict, differential, differential
 *     strict)
 * @param bus the follower's bus, or empty for the leader's bus
 * @param factory how to build the follower, or empty to use the leader's factory
 */
public record FollowerMotor(
    int canId,
    MotorAlignment alignment,
    FollowType followType,
    Optional<CANBusId> bus,
    Optional<MotorIOFactory> factory) {

  /** The type of follower tracking. */
  public enum FollowType {
    /** Follows the leader's output, honoring inversion relative to leader. */
    STANDARD,
    /** Follows the leader's output exactly, ignoring inversion. */
    STRICT,
    /** Follows the leader's differential output, honoring inversion relative to leader. */
    DIFFERENTIAL,
    /** Follows the leader's differential output exactly, ignoring inversion. */
    DIFFERENTIAL_STRICT
  }

  public FollowerMotor {
    Objects.requireNonNull(alignment, "alignment");
    Objects.requireNonNull(followType, "followType");
    Objects.requireNonNull(bus, "bus");
    Objects.requireNonNull(factory, "factory");
  }

  /**
   * Creates a follower with standard follow type on the specified bus and factory.
   *
   * @param canId the follower's CAN ID
   * @param alignment the follower's alignment to the leader
   * @param bus the follower's bus
   * @param factory the follower's factory
   */
  public FollowerMotor(
      int canId,
      MotorAlignment alignment,
      Optional<CANBusId> bus,
      Optional<MotorIOFactory> factory) {
    this(canId, alignment, FollowType.STANDARD, bus, factory);
  }

  /**
   * Creates a follower on the leader's bus, built by the leader's factory with standard follow
   * type.
   *
   * @param canId the follower's CAN ID
   * @param alignment the follower's alignment to the leader
   */
  public FollowerMotor(int canId, MotorAlignment alignment) {
    this(canId, alignment, FollowType.STANDARD, Optional.empty(), Optional.empty());
  }

  /**
   * Creates a follower on the leader's bus, built by the leader's factory with the given follow
   * type.
   *
   * @param canId the follower's CAN ID
   * @param alignment the follower's alignment to the leader
   * @param followType the follow type
   */
  public FollowerMotor(int canId, MotorAlignment alignment, FollowType followType) {
    this(canId, alignment, followType, Optional.empty(), Optional.empty());
  }

  /**
   * Returns this follower on another bus.
   *
   * @param bus the follower's bus
   * @return the updated follower
   */
  public FollowerMotor onBus(CANBusId bus) {
    return new FollowerMotor(canId, alignment, followType, Optional.of(bus), factory);
  }

  /**
   * Returns this follower built by another factory, e.g. a different vendor.
   *
   * @param factory the follower's factory
   * @return the updated follower
   */
  public FollowerMotor builtBy(MotorIOFactory factory) {
    return new FollowerMotor(canId, alignment, followType, bus, Optional.of(factory));
  }

  /**
   * Returns this follower with another follow type.
   *
   * @param followType the follow type
   * @return the updated follower
   */
  public FollowerMotor withFollowType(FollowType followType) {
    return new FollowerMotor(canId, alignment, followType, bus, factory);
  }
}
