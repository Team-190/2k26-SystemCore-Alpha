package edu.wpi.team190.gompeilib.core.io.base;

/** Capability interface for hardware components that report connection status. */
public interface ConnectedIO {
  /**
   * Reports whether the device is currently connected and communicating.
   *
   * @return true if connected, false otherwise
   */
  default boolean isConnected() {
    return false;
  }
}
