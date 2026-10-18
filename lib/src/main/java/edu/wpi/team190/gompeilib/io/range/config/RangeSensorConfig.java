package edu.wpi.team190.gompeilib.io.range.config;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import java.util.Set;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Frequency;

/** Everything one range (time-of-flight) sensor needs. */
@Builder(setterPrefix = "with", toBuilder = true)
public class RangeSensorConfig {
  /**
   * The {@link edu.wpi.team190.gompeilib.io.range.inputs.RangeSensorTelemetry} fields published at
   * the control rate.
   */
  public static final Set<String> CONTROL_FIELDS =
      Set.of("distance", "distanceStdDev", "detected", "signalStrength", "measurementHealth");

  /** The sensor's CAN ID. */
  @NonNull public final Integer canId;

  /** The bus the sensor is on. */
  @NonNull public final CANBusId bus;

  /** A target closer than this is detected, in [0, 4] m. */
  @NonNull @Builder.Default public final Distance proximityThreshold = Meters.of(0.4);

  /** How far past the threshold a detected target must move to be released, in [0, 1] m. */
  @NonNull @Builder.Default public final Distance proximityHysteresis = Meters.of(0.01);

  /** The weakest signal accepted as a valid measurement, in [1, 15000]. */
  @NonNull @Builder.Default public final Double minSignalStrength = 2500.0;

  /** How the sensor trades range for update rate. */
  @NonNull @Builder.Default
  public final RangeUpdateMode updateMode = RangeUpdateMode.SHORT_RANGE_100HZ;

  /** The update rate for the user-frequency modes, in [5, 50] Hz. */
  @NonNull @Builder.Default public final Frequency updateFrequency = Hertz.of(50.0);

  /** Center of the field of view along X, in [-11.8, 11.8] degrees. */
  @NonNull @Builder.Default public final Angle fovCenterX = Degrees.of(0.0);

  /** Center of the field of view along Y, in [-11.8, 11.8] degrees. */
  @NonNull @Builder.Default public final Angle fovCenterY = Degrees.of(0.0);

  /** Width of the field of view along X, in [6.75, 27] degrees. */
  @NonNull @Builder.Default public final Angle fovRangeX = Degrees.of(27.0);

  /** Width of the field of view along Y, in [6.75, 27] degrees. */
  @NonNull @Builder.Default public final Angle fovRangeY = Degrees.of(27.0);

  /** How often each telemetry field is published. */
  @NonNull @Builder.Default public final SignalRates signalRates = SignalRates.DEFAULT;
}
