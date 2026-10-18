package edu.wpi.team190.gompeilib.io.base;

/**
 * Capability interface for hardware that is commanded through control requests.
 *
 * <p>There is deliberately no default implementation: a vendor implementation must handle every
 * request in {@code R} (typically with an exhaustive {@code switch} over a sealed hierarchy) and
 * throw {@link UnsupportedOperationException} for any request the vendor cannot run, so a request
 * is never silently dropped.
 *
 * @param <R> the sealed request hierarchy this hardware accepts
 */
public interface RequestControllableIO<R> {
  /**
   * Sends a control request to the hardware.
   *
   * @param request the request to apply
   */
  void setControl(R request);
}
