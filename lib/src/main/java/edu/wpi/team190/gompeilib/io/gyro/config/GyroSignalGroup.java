package edu.wpi.team190.gompeilib.io.gyro.config;

import static org.wpilib.units.Units.Hertz;

import edu.wpi.team190.gompeilib.io.base.SignalGroup;
import org.wpilib.units.measure.Frequency;

/**
 * The groups of {@link edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetry} signals, each
 * published at its own rate.
 */
public enum GyroSignalGroup implements SignalGroup {
  /** Orientation and angular velocity. */
  CONTROL(100.0),

  /** Everything else. */
  DIAGNOSTIC(4.0);

  private final Frequency defaultRate;

  GyroSignalGroup(double defaultHertz) {
    this.defaultRate = Hertz.of(defaultHertz);
  }

  @Override
  public Frequency defaultRate() {
    return defaultRate;
  }
}
