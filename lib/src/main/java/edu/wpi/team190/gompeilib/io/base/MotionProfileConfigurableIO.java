package edu.wpi.team190.gompeilib.io.base;

import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;

/**
 * Capability interface for hardware that stores the constraints its profiled requests use when the
 * request doesn't carry its own.
 */
public interface MotionProfileConfigurableIO {
  /**
   * Updates the stored motion-profile constraints.
   *
   * @param motionProfile the new constraints
   */
  default void updateMotionProfile(MotionProfileConfig motionProfile) {}
}
