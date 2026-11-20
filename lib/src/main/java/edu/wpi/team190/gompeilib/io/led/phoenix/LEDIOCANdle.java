package edu.wpi.team190.gompeilib.io.led.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.team190.gompeilib.io.led.config.LEDConfig;
import edu.wpi.team190.gompeilib.io.led.config.LEDSignal;
import edu.wpi.team190.gompeilib.io.led.config.LEDSignalGroup;
import edu.wpi.team190.gompeilib.io.led.inputs.LEDTelemetry;
import edu.wpi.team190.gompeilib.io.led.interfaces.LEDIO;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRequest;
import edu.wpi.team190.gompeilib.io.phoenix.CANBuses;
import edu.wpi.team190.gompeilib.io.phoenix.FaultSignals;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixUtil;
import edu.wpi.team190.gompeilib.io.phoenix.SignalTracker;
import edu.wpi.team190.gompeilib.logging.Trace;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

/** A CANdle LED controller. Logs every status signal the device publishes. */
public class LEDIOCANdle implements LEDIO {
  private final CANdle candle;

  private final StatusSignal<Integer> version;
  private final StatusSignal<Boolean> isProLicensed;
  private final StatusSignal<Voltage> supplyVoltage;
  private final StatusSignal<Voltage> fiveVoltRailVoltage;
  private final StatusSignal<Current> outputCurrent;
  private final StatusSignal<Temperature> deviceTemperature;
  private final StatusSignal<Double> supplyModulation;
  private final StatusSignal<Integer> maxSimultaneousAnimations;
  private final FaultSignals<CANdleFault> faults;

  private boolean connected = false;

  /**
   * Creates and configures a CANdle.
   *
   * @param config the controller's config
   */
  public LEDIOCANdle(LEDConfig config) {
    CANBus bus = CANBuses.toCANBus(config.bus);
    candle = new CANdle(config.canId, bus);
    PhoenixUtil.tryUntilOk(
        5, () -> candle.getConfigurator().apply(CANdleConversions.toConfiguration(config), 0.25));

    SignalTracker<LEDSignal, LEDSignalGroup> signals = new SignalTracker<>(config.signalRates);
    version = signals.track(LEDSignal.FIRMWARE_VERSION, candle.getVersion());
    isProLicensed = signals.track(LEDSignal.IS_PRO_LICENSED, candle.getIsProLicensed());
    supplyVoltage = signals.track(LEDSignal.SUPPLY_VOLTAGE, candle.getSupplyVoltage());
    fiveVoltRailVoltage =
        signals.track(LEDSignal.FIVE_VOLT_RAIL_VOLTAGE, candle.getFiveVRailVoltage());
    outputCurrent = signals.track(LEDSignal.OUTPUT_CURRENT, candle.getOutputCurrent());
    deviceTemperature = signals.track(LEDSignal.DEVICE_TEMPERATURE, candle.getDeviceTemp());
    supplyModulation = signals.track(LEDSignal.SUPPLY_MODULATION, candle.getVBatModulation());
    maxSimultaneousAnimations =
        signals.track(
            LEDSignal.MAX_SIMULTANEOUS_ANIMATIONS, candle.getMaxSimultaneousAnimationCount());
    faults =
        signals.trackFaults(candle, CANdleFault.class, LEDSignal.FAULTS, LEDSignal.STICKY_FAULTS);
    signals.apply(candle, bus);
  }

  @Override
  @Trace
  public void updateInputs(LEDTelemetry telemetry) {
    connected = BaseStatusSignal.isAllGood(supplyVoltage, outputCurrent);
    telemetry.connected = connected;
    telemetry.firmwareVersion = version.getValue();
    telemetry.isProLicensed = isProLicensed.getValue();
    telemetry.supplyVoltage = supplyVoltage.getValue();
    telemetry.fiveVoltRailVoltage = fiveVoltRailVoltage.getValue();
    telemetry.outputCurrent = outputCurrent.getValue();
    telemetry.deviceTemperature = deviceTemperature.getValue();
    telemetry.supplyModulation = supplyModulation.getValue();
    telemetry.maxSimultaneousAnimations = maxSimultaneousAnimations.getValue();
    telemetry.faults = faults.faults();
    telemetry.stickyFaults = faults.stickyFaults();
  }

  @Override
  public boolean isConnected() {
    return connected;
  }

  @Override
  public void setControl(LEDRequest request) {
    candle.setControl(CANdleConversions.toControlRequest(request));
  }

  @Override
  public void clearStickyFaults() {
    candle.clearStickyFaults();
  }
}
