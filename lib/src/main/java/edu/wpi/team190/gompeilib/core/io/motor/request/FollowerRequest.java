package edu.wpi.team190.gompeilib.core.io.motor.request;

import edu.wpi.team190.gompeilib.core.io.motor.MotorAlignment;
import org.wpilib.units.measure.Frequency;

/** A request that makes this motor follow another motor controller on the same bus. */
public sealed interface FollowerRequest extends MotorRequest {
  /**
   * Returns the CAN ID of the leader.
   *
   * @return the leader's CAN ID
   */
  int leaderId();

  /**
   * Returns how often the request is re-sent.
   *
   * @return the update frequency
   */
  Frequency updateFrequency();

  /**
   * Follows the leader's output, honoring this motor's own inversion relative to the leader.
   *
   * @param leaderId the leader's CAN ID
   * @param alignment whether this motor's output is aligned with or opposed to the leader's
   * @param updateFrequency how often the request is re-sent
   */
  record Follow(int leaderId, MotorAlignment alignment, Frequency updateFrequency)
      implements FollowerRequest {
    public Follow(int leaderId, MotorAlignment alignment) {
      this(leaderId, alignment, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    }
  }

  /**
   * Follows the leader's output exactly, ignoring this motor's inversion settings.
   *
   * @param leaderId the leader's CAN ID
   * @param updateFrequency how often the request is re-sent
   */
  record StrictFollow(int leaderId, Frequency updateFrequency) implements FollowerRequest {
    public StrictFollow(int leaderId) {
      this(leaderId, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    }
  }

  /**
   * Follows the leader's differential output, honoring this motor's inversion relative to the
   * leader.
   *
   * @param leaderId the leader's CAN ID
   * @param alignment whether this motor's output is aligned with or opposed to the leader's
   * @param updateFrequency how often the request is re-sent
   */
  record DifferentialFollow(int leaderId, MotorAlignment alignment, Frequency updateFrequency)
      implements FollowerRequest {
    public DifferentialFollow(int leaderId, MotorAlignment alignment) {
      this(leaderId, alignment, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    }
  }

  /**
   * Follows the leader's differential output exactly, ignoring this motor's inversion settings.
   *
   * @param leaderId the leader's CAN ID
   * @param updateFrequency how often the request is re-sent
   */
  record DifferentialStrictFollow(int leaderId, Frequency updateFrequency)
      implements FollowerRequest {
    public DifferentialStrictFollow(int leaderId) {
      this(leaderId, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    }
  }
}
