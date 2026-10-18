package edu.wpi.team190.gompeilib.io.gyro.phoenix;

import edu.wpi.team190.gompeilib.io.base.HardwareFault;

/**
 * CTRE Pigeon2 fault bits, in the order Phoenix6 declares the device's fault accessors.
 *
 * <p>The bit positions are GompeiLib's own: the Pigeon2 implementation packs them from the
 * individual fault accessors rather than from the device's raw fault field.
 */
public enum Pigeon2Fault implements HardwareFault {
  HARDWARE(0),
  UNDERVOLTAGE(1),
  BOOT_DURING_ENABLE(2),
  UNLICENSED_FEATURE_IN_USE(3),
  BOOTUP_ACCELEROMETER(4),
  BOOTUP_GYROSCOPE(5),
  BOOTUP_MAGNETOMETER(6),
  BOOT_INTO_MOTION(7),
  DATA_ACQUIRED_LATE(8),
  LOOP_TIME_SLOW(9),
  SATURATED_MAGNETOMETER(10),
  SATURATED_ACCELEROMETER(11),
  SATURATED_GYROSCOPE(12);

  private final int bit;

  Pigeon2Fault(int bit) {
    this.bit = bit;
  }

  @Override
  public int bit() {
    return bit;
  }
}
