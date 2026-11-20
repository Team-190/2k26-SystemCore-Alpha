package edu.wpi.team190.gompeilib.io.range.config;

import edu.wpi.team190.gompeilib.io.base.Signal;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.range.inputs.RangeSensorTelemetry;

/**
 * The status signals {@link RangeSensorTelemetry} is filled from, each in one {@link
 * RangeSensorSignalGroup}.
 */
public enum RangeSensorSignal implements Signal<RangeSensorSignalGroup> {
  /** {@link RangeSensorTelemetry#distance}. */
  DISTANCE(RangeSensorSignalGroup.CONTROL),

  /** {@link RangeSensorTelemetry#distanceStdDev}. */
  DISTANCE_STD_DEV(RangeSensorSignalGroup.CONTROL),

  /** {@link RangeSensorTelemetry#detected}. */
  DETECTED(RangeSensorSignalGroup.CONTROL),

  /** {@link RangeSensorTelemetry#signalStrength}. */
  SIGNAL_STRENGTH(RangeSensorSignalGroup.CONTROL),

  /** {@link RangeSensorTelemetry#measurementHealth}. */
  MEASUREMENT_HEALTH(RangeSensorSignalGroup.CONTROL),

  /** {@link RangeSensorTelemetry#isProLicensed}. */
  IS_PRO_LICENSED(RangeSensorSignalGroup.DIAGNOSTIC),

  /** {@link RangeSensorTelemetry#firmwareVersion}. */
  FIRMWARE_VERSION(RangeSensorSignalGroup.DIAGNOSTIC),

  /** {@link RangeSensorTelemetry#measurementTime}. */
  MEASUREMENT_TIME(RangeSensorSignalGroup.DIAGNOSTIC),

  /** {@link RangeSensorTelemetry#ambientSignal}. */
  AMBIENT_SIGNAL(RangeSensorSignalGroup.DIAGNOSTIC),

  /** {@link RangeSensorTelemetry#fovCenterX}. */
  FOV_CENTER_X(RangeSensorSignalGroup.DIAGNOSTIC),

  /** {@link RangeSensorTelemetry#fovCenterY}. */
  FOV_CENTER_Y(RangeSensorSignalGroup.DIAGNOSTIC),

  /** {@link RangeSensorTelemetry#fovRangeX}. */
  FOV_RANGE_X(RangeSensorSignalGroup.DIAGNOSTIC),

  /** {@link RangeSensorTelemetry#fovRangeY}. */
  FOV_RANGE_Y(RangeSensorSignalGroup.DIAGNOSTIC),

  /** {@link RangeSensorTelemetry#supplyVoltage}. */
  SUPPLY_VOLTAGE(RangeSensorSignalGroup.DIAGNOSTIC),

  /** {@link RangeSensorTelemetry#faults}. */
  FAULTS(RangeSensorSignalGroup.DIAGNOSTIC),

  /** {@link RangeSensorTelemetry#stickyFaults}. */
  STICKY_FAULTS(RangeSensorSignalGroup.DIAGNOSTIC);

  /** Every group at its default rate, with no signal overrides. */
  public static final SignalRates<RangeSensorSignal, RangeSensorSignalGroup> DEFAULT_RATES =
      SignalRates.defaults(RangeSensorSignal.class, RangeSensorSignalGroup.class);

  private final RangeSensorSignalGroup group;

  RangeSensorSignal(RangeSensorSignalGroup group) {
    this.group = group;
  }

  @Override
  public RangeSensorSignalGroup group() {
    return group;
  }
}
