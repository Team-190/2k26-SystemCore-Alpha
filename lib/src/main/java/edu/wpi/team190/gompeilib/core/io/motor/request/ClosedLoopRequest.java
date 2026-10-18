package edu.wpi.team190.gompeilib.core.io.motor.request;

import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;

/** A request that closes the loop on position or velocity. */
public sealed interface ClosedLoopRequest extends AxisRequest
    permits PositionRequest, VelocityRequest {
  /**
   * Returns the gain slot the closed loop uses.
   *
   * @return the gain slot
   */
  GainSlot slot();
}
