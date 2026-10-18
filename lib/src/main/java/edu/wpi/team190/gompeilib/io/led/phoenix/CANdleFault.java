package edu.wpi.team190.gompeilib.io.led.phoenix;

import edu.wpi.team190.gompeilib.io.base.HardwareFault;

/**
 * CTRE CANdle fault bits, in the order Phoenix6 declares the device's fault accessors.
 *
 * <p>The bit positions are GompeiLib's own: the CANdle implementation packs them from the
 * individual fault accessors rather than from the device's raw fault field.
 */
public enum CANdleFault implements HardwareFault {
  HARDWARE(0),
  UNDERVOLTAGE(1),
  BOOT_DURING_ENABLE(2),
  UNLICENSED_FEATURE_IN_USE(3),
  OVERVOLTAGE(4),
  THERMAL(5),
  SOFTWARE_FUSE(6),
  SHORT_CIRCUIT(7),
  FIVE_V_TOO_HIGH(8),
  FIVE_V_TOO_LOW(9);

  private final int bit;

  CANdleFault(int bit) {
    this.bit = bit;
  }

  @Override
  public int bit() {
    return bit;
  }
}
