package edu.wpi.team190.gompeilib.core.io.encoder.phoenix;

import edu.wpi.team190.gompeilib.core.io.base.HardwareFault;

/**
 * CTRE CANcoder fault bits, in the order Phoenix6 declares the device's fault accessors.
 *
 * <p>The bit positions are GompeiLib's own: the CANcoder implementation packs them from the
 * individual fault accessors rather than from the device's raw fault field.
 */
public enum CANcoderFault implements HardwareFault {
  HARDWARE(0),
  UNDERVOLTAGE(1),
  BOOT_DURING_ENABLE(2),
  UNLICENSED_FEATURE_IN_USE(3),
  BAD_MAGNET(4);

  private final int bit;

  CANcoderFault(int bit) {
    this.bit = bit;
  }

  @Override
  public int bit() {
    return bit;
  }
}
