package edu.wpi.team190.gompeilib.io.encoder.config;

import static org.wpilib.units.Units.Rotations;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.units.measure.Angle;

/** Everything one absolute encoder needs. */
@Builder(setterPrefix = "with", toBuilder = true)
public class AbsoluteEncoderConfig {
  /** The encoder's CAN ID. */
  @NonNull public final Integer canId;

  /** The bus the encoder is on. */
  @NonNull public final CANBusId bus;

  /** Which magnet rotation is positive. */
  @NonNull @Builder.Default
  public final SensorDirection direction = SensorDirection.COUNTER_CLOCKWISE_POSITIVE;

  /** Added to the raw absolute position, in [-1, 1] rotations. */
  @NonNull @Builder.Default public final Angle magnetOffset = Rotations.of(0.0);

  /**
   * Where the absolute position wraps, in [0, 1] rotations: it is reported in {@code
   * [discontinuityPoint - 1, discontinuityPoint)}.
   */
  @NonNull @Builder.Default public final Angle discontinuityPoint = Rotations.of(0.5);

  /** How often each telemetry field is published. */
  @NonNull @Builder.Default
  public final SignalRates<AbsoluteEncoderSignal, AbsoluteEncoderSignalGroup> signalRates =
      AbsoluteEncoderSignal.DEFAULT_RATES;
}
