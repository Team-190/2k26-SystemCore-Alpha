package edu.wpi.team190.gompeilib.core.io.motor.phoenix;

import edu.wpi.team190.gompeilib.core.io.base.HardwareFault;

/**
 * CTRE TalonFX fault bits, in the order Phoenix6 declares the device's fault accessors.
 *
 * <p>The bit positions are GompeiLib's own: the TalonFX implementation packs them from the
 * individual fault accessors rather than from the device's raw fault field.
 */
public enum TalonFXFault implements HardwareFault {
  HARDWARE(0),
  PROC_TEMP(1),
  DEVICE_TEMP(2),
  UNDERVOLTAGE(3),
  BOOT_DURING_ENABLE(4),
  UNLICENSED_FEATURE_IN_USE(5),
  BRIDGE_BROWNOUT(6),
  REMOTE_SENSOR_RESET(7),
  MISSING_DIFFERENTIAL_FX(8),
  REMOTE_SENSOR_POS_OVERFLOW(9),
  OVER_SUPPLY_V(10),
  UNSTABLE_SUPPLY_V(11),
  REVERSE_HARD_LIMIT(12),
  FORWARD_HARD_LIMIT(13),
  REVERSE_SOFT_LIMIT(14),
  FORWARD_SOFT_LIMIT(15),
  MISSING_SOFT_LIMIT_REMOTE(16),
  MISSING_HARD_LIMIT_REMOTE(17),
  REMOTE_SENSOR_DATA_INVALID(18),
  FUSED_SENSOR_OUT_OF_SYNC(19),
  STATOR_CURR_LIMIT(20),
  SUPPLY_CURR_LIMIT(21),
  USING_FUSED_CANCODER_WHILE_UNLICENSED(22),
  STATIC_BRAKE_DISABLED(23),
  ROTOR_FAULT_1(24),
  ROTOR_FAULT_2(25);

  private final int bit;

  TalonFXFault(int bit) {
    this.bit = bit;
  }

  @Override
  public int bit() {
    return bit;
  }
}
