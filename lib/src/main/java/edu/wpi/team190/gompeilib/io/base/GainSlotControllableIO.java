package edu.wpi.team190.gompeilib.io.base;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;

/**
 * Capability interface for hardware that stores closed-loop gains in slots. The slot a request uses
 * travels on the request itself.
 */
public interface GainSlotControllableIO {
  /**
   * Updates the closed-loop gains for a specific gain slot.
   *
   * @param gains the gains to apply
   * @param gainSlot the target gain slot
   */
  default void updateGains(Gains gains, GainSlot gainSlot) {}
}
