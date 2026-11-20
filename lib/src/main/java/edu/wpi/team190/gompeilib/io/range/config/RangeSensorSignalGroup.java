package edu.wpi.team190.gompeilib.io.range.config;

import static org.wpilib.units.Units.Hertz;

import edu.wpi.team190.gompeilib.io.base.SignalGroup;
import org.wpilib.units.measure.Frequency;

/**
 * The groups of {@link edu.wpi.team190.gompeilib.io.range.inputs.RangeSensorTelemetry} signals,
 * each published at its own rate.
 */
public enum RangeSensorSignalGroup implements SignalGroup {
  /** The measurement and its quality. */
  CONTROL(100.0),

  /** Everything else. */
  DIAGNOSTIC(4.0);

  private final Frequency defaultRate;

  RangeSensorSignalGroup(double defaultHertz) {
    this.defaultRate = Hertz.of(defaultHertz);
  }

  @Override
  public Frequency defaultRate() {
    return defaultRate;
  }
}
