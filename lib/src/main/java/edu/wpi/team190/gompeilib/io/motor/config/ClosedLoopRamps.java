package edu.wpi.team190.gompeilib.io.motor.config;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Time;

/**
 * How long closed-loop output takes to ramp from zero to full, per output type. Zero disables the
 * ramp.
 *
 * @param dutyCycle the duty-cycle ramp period
 * @param voltage the voltage ramp period
 * @param current the current ramp period
 */
public record ClosedLoopRamps(Time dutyCycle, Time voltage, Time current) {
  /** No ramping. */
  public static final ClosedLoopRamps NONE =
      new ClosedLoopRamps(Seconds.of(0.0), Seconds.of(0.0), Seconds.of(0.0));
}
