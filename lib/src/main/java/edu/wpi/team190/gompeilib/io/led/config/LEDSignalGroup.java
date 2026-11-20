package edu.wpi.team190.gompeilib.io.led.config;

import static org.wpilib.units.Units.Hertz;

import edu.wpi.team190.gompeilib.io.base.SignalGroup;
import org.wpilib.units.measure.Frequency;

/**
 * The groups of {@link edu.wpi.team190.gompeilib.io.led.inputs.LEDTelemetry} signals, each
 * published at its own rate.
 */
public enum LEDSignalGroup implements SignalGroup {
  /** All LED telemetry, which is diagnostic. */
  DIAGNOSTIC(4.0);

  private final Frequency defaultRate;

  LEDSignalGroup(double defaultHertz) {
    this.defaultRate = Hertz.of(defaultHertz);
  }

  @Override
  public Frequency defaultRate() {
    return defaultRate;
  }
}
