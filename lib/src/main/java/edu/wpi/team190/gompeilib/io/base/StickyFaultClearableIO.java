package edu.wpi.team190.gompeilib.io.base;

/** Capability interface for hardware that latches sticky faults. */
public interface StickyFaultClearableIO {
  /** Clears all sticky faults on the device. */
  default void clearStickyFaults() {}
}
