package edu.wpi.team190.gompeilib.io.led.config;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import java.util.Set;
import lombok.Builder;
import lombok.NonNull;

/** Everything one LED controller needs. */
@Builder(setterPrefix = "with", toBuilder = true)
public class LEDConfig {
  /**
   * The {@link edu.wpi.team190.gompeilib.io.led.inputs.LEDTelemetry} fields published at the
   * control rate. LED telemetry is all diagnostic.
   */
  public static final Set<String> CONTROL_FIELDS = Set.of();

  /** The controller's CAN ID. */
  @NonNull public final Integer canId;

  /** The bus the controller is on. */
  @NonNull public final CANBusId bus;

  /** The strip's color channel order. */
  @NonNull @Builder.Default public final LEDStripType stripType = LEDStripType.GRB;

  /** Scales every LED's brightness, in [0, 1]. */
  @NonNull @Builder.Default public final Double brightness = 1.0;

  /** Whether animations keep running when the controller loses its CAN signal. */
  @NonNull @Builder.Default public final Boolean keepRunningOnLossOfSignal = true;

  /** Whether the controller's 5 V rail is enabled. */
  @NonNull @Builder.Default public final Boolean enableFiveVoltRail = true;

  /** What the supply-voltage output does. */
  @NonNull @Builder.Default public final SupplyOutputMode supplyOutputMode = SupplyOutputMode.ON;

  /** Whether the controller's own status LEDs light while it is controlled. */
  @NonNull @Builder.Default public final Boolean statusLedWhenActive = true;

  /** How often each telemetry field is published. */
  @NonNull @Builder.Default public final SignalRates signalRates = SignalRates.DEFAULT;
}
