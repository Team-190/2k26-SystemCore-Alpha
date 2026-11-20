package edu.wpi.team190.gompeilib.io.base;

/**
 * One status signal a device publishes, filling the telemetry field of the same name. Each device
 * defines its signals as an enum implementing this interface, and each signal belongs to one of the
 * device's {@link SignalGroup}s.
 *
 * @param <G> the device's group enum
 */
public interface Signal<G extends Enum<G> & SignalGroup> {
  /**
   * Returns the group this signal is published with.
   *
   * @return the group
   */
  G group();
}
