package edu.wpi.team190.gompeilib.io.base;

import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;

/** Capability interface for hardware whose current limits can change while running. */
public interface CurrentLimitConfigurableIO {
  /**
   * Updates the current limits.
   *
   * @param currentLimits the new limits
   */
  default void updateCurrentLimits(CurrentLimits currentLimits) {}
}
