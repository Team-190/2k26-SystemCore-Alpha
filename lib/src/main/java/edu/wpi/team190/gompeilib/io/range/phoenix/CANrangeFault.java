package edu.wpi.team190.gompeilib.io.range.phoenix;

import edu.wpi.team190.gompeilib.io.base.HardwareFault;

/**
 * CTRE CANrange fault bits, in the order Phoenix6 declares the device's fault accessors.
 *
 * <p>The bit positions are GompeiLib's own: the CANrange implementation packs them from the
 * individual fault accessors rather than from the device's raw fault field.
 */
public enum CANrangeFault implements HardwareFault {
  HARDWARE(0),
  UNDERVOLTAGE(1),
  BOOT_DURING_ENABLE(2),
  UNLICENSED_FEATURE_IN_USE(3);

  private final int bit;

  CANrangeFault(int bit) {
    this.bit = bit;
  }

  @Override
  public int bit() {
    return bit;
  }
}
