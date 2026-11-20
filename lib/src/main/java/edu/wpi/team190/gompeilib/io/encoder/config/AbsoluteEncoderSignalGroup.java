package edu.wpi.team190.gompeilib.io.encoder.config;

import static org.wpilib.units.Units.Hertz;

import edu.wpi.team190.gompeilib.io.base.SignalGroup;
import org.wpilib.units.measure.Frequency;

/**
 * The groups of {@link edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry}
 * signals, each published at its own rate.
 */
public enum AbsoluteEncoderSignalGroup implements SignalGroup {
  /** Position and velocity. */
  CONTROL(100.0),

  /** Everything else. */
  DIAGNOSTIC(4.0);

  private final Frequency defaultRate;

  AbsoluteEncoderSignalGroup(double defaultHertz) {
    this.defaultRate = Hertz.of(defaultHertz);
  }

  @Override
  public Frequency defaultRate() {
    return defaultRate;
  }
}
