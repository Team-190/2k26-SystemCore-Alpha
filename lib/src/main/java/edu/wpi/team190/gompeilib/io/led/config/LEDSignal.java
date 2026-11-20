package edu.wpi.team190.gompeilib.io.led.config;

import edu.wpi.team190.gompeilib.io.base.Signal;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.led.inputs.LEDTelemetry;

/** The status signals {@link LEDTelemetry} is filled from, each in one {@link LEDSignalGroup}. */
public enum LEDSignal implements Signal<LEDSignalGroup> {
  /** {@link LEDTelemetry#isProLicensed}. */
  IS_PRO_LICENSED(LEDSignalGroup.DIAGNOSTIC),

  /** {@link LEDTelemetry#firmwareVersion}. */
  FIRMWARE_VERSION(LEDSignalGroup.DIAGNOSTIC),

  /** {@link LEDTelemetry#supplyVoltage}. */
  SUPPLY_VOLTAGE(LEDSignalGroup.DIAGNOSTIC),

  /** {@link LEDTelemetry#fiveVoltRailVoltage}. */
  FIVE_VOLT_RAIL_VOLTAGE(LEDSignalGroup.DIAGNOSTIC),

  /** {@link LEDTelemetry#outputCurrent}. */
  OUTPUT_CURRENT(LEDSignalGroup.DIAGNOSTIC),

  /** {@link LEDTelemetry#deviceTemperature}. */
  DEVICE_TEMPERATURE(LEDSignalGroup.DIAGNOSTIC),

  /** {@link LEDTelemetry#supplyModulation}. */
  SUPPLY_MODULATION(LEDSignalGroup.DIAGNOSTIC),

  /** {@link LEDTelemetry#maxSimultaneousAnimations}. */
  MAX_SIMULTANEOUS_ANIMATIONS(LEDSignalGroup.DIAGNOSTIC),

  /** {@link LEDTelemetry#faults}. */
  FAULTS(LEDSignalGroup.DIAGNOSTIC),

  /** {@link LEDTelemetry#stickyFaults}. */
  STICKY_FAULTS(LEDSignalGroup.DIAGNOSTIC);

  /** Every group at its default rate, with no signal overrides. */
  public static final SignalRates<LEDSignal, LEDSignalGroup> DEFAULT_RATES =
      SignalRates.defaults(LEDSignal.class, LEDSignalGroup.class);

  private final LEDSignalGroup group;

  LEDSignal(LEDSignalGroup group) {
    this.group = group;
  }

  @Override
  public LEDSignalGroup group() {
    return group;
  }
}
