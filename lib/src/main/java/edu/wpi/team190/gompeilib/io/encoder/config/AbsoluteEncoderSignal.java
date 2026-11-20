package edu.wpi.team190.gompeilib.io.encoder.config;

import edu.wpi.team190.gompeilib.io.base.Signal;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry;

/**
 * The status signals {@link AbsoluteEncoderTelemetry} is filled from, each in one {@link
 * AbsoluteEncoderSignalGroup}.
 */
public enum AbsoluteEncoderSignal implements Signal<AbsoluteEncoderSignalGroup> {
  /** {@link AbsoluteEncoderTelemetry#absolutePosition}. */
  ABSOLUTE_POSITION(AbsoluteEncoderSignalGroup.CONTROL),

  /** {@link AbsoluteEncoderTelemetry#position}. */
  POSITION(AbsoluteEncoderSignalGroup.CONTROL),

  /** {@link AbsoluteEncoderTelemetry#positionSinceBoot}. */
  POSITION_SINCE_BOOT(AbsoluteEncoderSignalGroup.CONTROL),

  /** {@link AbsoluteEncoderTelemetry#velocity}. */
  VELOCITY(AbsoluteEncoderSignalGroup.CONTROL),

  /** {@link AbsoluteEncoderTelemetry#unfilteredVelocity}. */
  UNFILTERED_VELOCITY(AbsoluteEncoderSignalGroup.CONTROL),

  /** {@link AbsoluteEncoderTelemetry#isProLicensed}. */
  IS_PRO_LICENSED(AbsoluteEncoderSignalGroup.DIAGNOSTIC),

  /** {@link AbsoluteEncoderTelemetry#firmwareVersion}. */
  FIRMWARE_VERSION(AbsoluteEncoderSignalGroup.DIAGNOSTIC),

  /** {@link AbsoluteEncoderTelemetry#magnetHealth}. */
  MAGNET_HEALTH(AbsoluteEncoderSignalGroup.DIAGNOSTIC),

  /** {@link AbsoluteEncoderTelemetry#supplyVoltage}. */
  SUPPLY_VOLTAGE(AbsoluteEncoderSignalGroup.DIAGNOSTIC),

  /** {@link AbsoluteEncoderTelemetry#faults}. */
  FAULTS(AbsoluteEncoderSignalGroup.DIAGNOSTIC),

  /** {@link AbsoluteEncoderTelemetry#stickyFaults}. */
  STICKY_FAULTS(AbsoluteEncoderSignalGroup.DIAGNOSTIC);

  /** Every group at its default rate, with no signal overrides. */
  public static final SignalRates<AbsoluteEncoderSignal, AbsoluteEncoderSignalGroup> DEFAULT_RATES =
      SignalRates.defaults(AbsoluteEncoderSignal.class, AbsoluteEncoderSignalGroup.class);

  private final AbsoluteEncoderSignalGroup group;

  AbsoluteEncoderSignal(AbsoluteEncoderSignalGroup group) {
    this.group = group;
  }

  @Override
  public AbsoluteEncoderSignalGroup group() {
    return group;
  }
}
