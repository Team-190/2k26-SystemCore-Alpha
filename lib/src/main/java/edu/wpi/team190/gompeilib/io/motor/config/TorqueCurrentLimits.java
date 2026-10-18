package edu.wpi.team190.gompeilib.io.motor.config;

import static org.wpilib.units.Units.Amps;

import java.util.Objects;
import org.wpilib.units.measure.Current;

/**
 * The most torque-producing current a motor may apply in each direction, in any current-output
 * request.
 *
 * @param peakForward the forward peak, zero or positive
 * @param peakReverse the reverse peak, zero or negative
 */
public record TorqueCurrentLimits(Current peakForward, Current peakReverse) {
  /** The widest limits a motor accepts, so current output is limited only by the hardware. */
  public static final TorqueCurrentLimits NONE =
      new TorqueCurrentLimits(Amps.of(800.0), Amps.of(-800.0));

  public TorqueCurrentLimits {
    Objects.requireNonNull(peakForward, "peakForward");
    Objects.requireNonNull(peakReverse, "peakReverse");
  }

  /**
   * Returns limits that are the same size in both directions.
   *
   * @param peak the peak, zero or positive
   * @return the limits
   */
  public static TorqueCurrentLimits symmetric(Current peak) {
    return new TorqueCurrentLimits(peak, peak.unaryMinus());
  }
}
