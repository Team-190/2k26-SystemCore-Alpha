package edu.wpi.team190.gompeilib.io.encoder.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.phoenix.CANBuses;
import edu.wpi.team190.gompeilib.io.phoenix.FaultSignals;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixUtil;
import edu.wpi.team190.gompeilib.io.phoenix.SignalTracker;
import edu.wpi.team190.gompeilib.logging.Trace;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

/** A CANcoder absolute encoder. Logs every status signal the device publishes. */
public class AbsoluteEncoderIOCANcoder implements AbsoluteEncoderIO {
  protected final CANcoder cancoder;

  private final StatusSignal<Integer> version;
  private final StatusSignal<Boolean> isProLicensed;
  private final StatusSignal<Angle> absolutePosition;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Angle> positionSinceBoot;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularVelocity> unfilteredVelocity;
  private final StatusSignal<MagnetHealthValue> magnetHealth;
  private final StatusSignal<Voltage> supplyVoltage;
  private final FaultSignals<CANcoderFault> faults;

  private boolean connected = false;

  /**
   * Creates and configures a CANcoder.
   *
   * @param config the encoder's config
   */
  public AbsoluteEncoderIOCANcoder(AbsoluteEncoderConfig config) {
    CANBus bus = CANBuses.toCANBus(config.bus);
    cancoder = new CANcoder(config.canId, bus);
    PhoenixUtil.tryUntilOk(
        5,
        () -> cancoder.getConfigurator().apply(CANcoderConversions.toConfiguration(config), 0.25));

    SignalTracker signals =
        new SignalTracker(
            config.signalRates,
            AbsoluteEncoderConfig.CONTROL_FIELDS,
            AbsoluteEncoderTelemetry.class);
    version = signals.track("firmwareVersion", cancoder.getVersion());
    isProLicensed = signals.track("isProLicensed", cancoder.getIsProLicensed());
    absolutePosition = signals.track("absolutePosition", cancoder.getAbsolutePosition());
    position = signals.track("position", cancoder.getPosition());
    positionSinceBoot = signals.track("positionSinceBoot", cancoder.getPositionSinceBoot());
    velocity = signals.track("velocity", cancoder.getVelocity());
    unfilteredVelocity = signals.track("unfilteredVelocity", cancoder.getUnfilteredVelocity());
    magnetHealth = signals.track("magnetHealth", cancoder.getMagnetHealth());
    supplyVoltage = signals.track("supplyVoltage", cancoder.getSupplyVoltage());
    faults = signals.trackFaults(cancoder, CANcoderFault.class);
    signals.apply(cancoder, bus);
  }

  @Override
  @Trace
  public void updateInputs(AbsoluteEncoderTelemetry telemetry) {
    connected = BaseStatusSignal.isAllGood(absolutePosition, position, velocity);
    telemetry.connected = connected;
    telemetry.firmwareVersion = version.getValue();
    telemetry.isProLicensed = isProLicensed.getValue();
    telemetry.absolutePosition = absolutePosition.getValue();
    telemetry.position = position.getValue();
    telemetry.positionSinceBoot = positionSinceBoot.getValue();
    telemetry.velocity = velocity.getValue();
    telemetry.unfilteredVelocity = unfilteredVelocity.getValue();
    telemetry.magnetHealth = CANcoderConversions.toMagnetHealth(magnetHealth.getValue());
    telemetry.supplyVoltage = supplyVoltage.getValue();
    telemetry.faults = faults.faults();
    telemetry.stickyFaults = faults.stickyFaults();
  }

  @Override
  public boolean isConnected() {
    return connected;
  }

  @Override
  public void setPosition(Angle position) {
    cancoder.setPosition(position);
  }

  @Override
  public void clearStickyFaults() {
    cancoder.clearStickyFaults();
  }
}
