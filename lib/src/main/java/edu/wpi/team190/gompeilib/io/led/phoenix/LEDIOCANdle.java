package edu.wpi.team190.gompeilib.io.led.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.team190.gompeilib.io.led.config.LEDConfig;
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

    SignalTracker signals =
        new SignalTracker(config.signalRates, LEDConfig.CONTROL_FIELDS, LEDTelemetry.class);
    version = signals.track("firmwareVersion", candle.getVersion());
    isProLicensed = signals.track("isProLicensed", candle.getIsProLicensed());
    supplyVoltage = signals.track("supplyVoltage", candle.getSupplyVoltage());
    fiveVoltRailVoltage = signals.track("fiveVoltRailVoltage", candle.getFiveVRailVoltage());
    outputCurrent = signals.track("outputCurrent", candle.getOutputCurrent());
    deviceTemperature = signals.track("deviceTemperature", candle.getDeviceTemp());
    supplyModulation = signals.track("supplyModulation", candle.getVBatModulation());
    maxSimultaneousAnimations =
        signals.track("maxSimultaneousAnimations", candle.getMaxSimultaneousAnimationCount());
    faults = signals.trackFaults(candle, CANdleFault.class);
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
