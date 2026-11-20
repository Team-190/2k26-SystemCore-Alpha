package edu.wpi.team190.gompeilib.io.motor.config;

import static org.wpilib.units.Units.Hertz;

import edu.wpi.team190.gompeilib.io.base.SignalGroup;
import org.wpilib.units.measure.Frequency;

/**
 * The groups of {@link edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry} signals, each
 * published at its own rate.
 */
public enum MotorSignalGroup implements SignalGroup {
  /** Mechanism and rotor motion. */
  FEEDBACK(100.0),

  /**
   * What the motor is applying. Hardware followers copy these, so a leader with hardware followers
   * must publish them at least {@link MotorSignalGroup#MIN_FOLLOWED_RATE}.
   */
  OUTPUT(100.0),

  /** Supply and stator current. */
  CURRENT(50.0),

  /** Limit switch states. */
  LIMITS(20.0),

  /** Which mode and slot the controller is running, and whether it's enabled. */
  CONTROL_STATE(10.0),

  /** Closed-loop targets, errors, and terms, and motion profile state. Raise while tuning. */
  CLOSED_LOOP(4.0),

  /** Temperatures. */
  THERMAL(4.0),

  /** Values that rarely or never change. */
  IDENTITY(4.0),

  /** Live and sticky faults. */
  FAULTS(4.0),

  /**
   * The difference axis of a differential mechanism. Off by default; differential mechanisms turn
   * it on. A leader with differential hardware followers must publish it at least {@link
   * MotorSignalGroup#MIN_FOLLOWED_RATE}.
   */
  DIFFERENTIAL(0.0);

  /** The slowest rate a group hardware followers copy from their leader may be published at. */
  public static final Frequency MIN_FOLLOWED_RATE = Hertz.of(20.0);

  private final Frequency defaultRate;

  MotorSignalGroup(double defaultHertz) {
    this.defaultRate = Hertz.of(defaultHertz);
  }

  @Override
  public Frequency defaultRate() {
    return defaultRate;
  }
}
