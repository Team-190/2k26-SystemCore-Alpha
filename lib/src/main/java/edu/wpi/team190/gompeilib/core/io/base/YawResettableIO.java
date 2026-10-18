package edu.wpi.team190.gompeilib.core.io.base;

import org.wpilib.units.measure.Angle;

/** Capability interface for gyros whose reported yaw can be reset. */
public interface YawResettableIO {
  /**
   * Sets the current yaw to the given value.
   *
   * @param yaw the new yaw
   */
  default void setYaw(Angle yaw) {}
}
