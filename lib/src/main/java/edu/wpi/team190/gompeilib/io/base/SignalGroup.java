package edu.wpi.team190.gompeilib.io.base;

import org.wpilib.units.measure.Frequency;

/**
 * A set of a device's {@link Signal}s that are published together at one rate. Each device defines
 * its groups as an enum implementing this interface; {@link SignalRates} sets each group's rate.
 */
public interface SignalGroup {
  /**
   * Returns the rate this group is published at unless configured otherwise. Zero turns the group
   * off.
   *
   * @return the default rate
   */
  Frequency defaultRate();
}
