package edu.wpi.team190.gompeilib.io.base;

import org.wpilib.units.measure.Angle;

/** Capability interface for hardware whose reported position can be reset. */
public interface PositionResettableIO {
  /**
   * Sets the current sensor position to the given value.
   *
   * @param position the new position
   */
  default void setPosition(Angle position) {}
}
