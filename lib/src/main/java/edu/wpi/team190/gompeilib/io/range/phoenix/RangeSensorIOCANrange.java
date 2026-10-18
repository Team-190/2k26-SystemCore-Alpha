package edu.wpi.team190.gompeilib.io.range.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import edu.wpi.team190.gompeilib.io.phoenix.CANBuses;
import edu.wpi.team190.gompeilib.io.phoenix.FaultSignals;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixUtil;
import edu.wpi.team190.gompeilib.io.phoenix.SignalTracker;
import edu.wpi.team190.gompeilib.io.range.config.RangeSensorConfig;
import edu.wpi.team190.gompeilib.io.range.inputs.RangeSensorTelemetry;
import edu.wpi.team190.gompeilib.io.range.interfaces.RangeSensorIO;
import edu.wpi.team190.gompeilib.logging.Trace;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;

/** A CANrange time-of-flight sensor. Logs every status signal the device publishes. */
public class RangeSensorIOCANrange implements RangeSensorIO {
  private final CANrange canrange;

  private final StatusSignal<Integer> version;
  private final StatusSignal<Boolean> isProLicensed;
  private final StatusSignal<Distance> distance;
  private final StatusSignal<Distance> distanceStdDev;
  private final StatusSignal<Time> measurementTime;
  private final StatusSignal<Boolean> detected;
  private final StatusSignal<Double> signalStrength;
  private final StatusSignal<Double> ambientSignal;
  private final StatusSignal<MeasurementHealthValue> measurementHealth;
  private final StatusSignal<Angle> fovCenterX;
  private final StatusSignal<Angle> fovCenterY;
  private final StatusSignal<Angle> fovRangeX;
  private final StatusSignal<Angle> fovRangeY;
  private final StatusSignal<Voltage> supplyVoltage;
  private final FaultSignals<CANrangeFault> faults;

  private boolean connected = false;

  /**
   * Creates and configures a CANrange.
   *
   * @param config the sensor's config
   */
  public RangeSensorIOCANrange(RangeSensorConfig config) {
    CANBus bus = CANBuses.toCANBus(config.bus);
    canrange = new CANrange(config.canId, bus);
    PhoenixUtil.tryUntilOk(
        5,
        () -> canrange.getConfigurator().apply(CANrangeConversions.toConfiguration(config), 0.25));

    SignalTracker signals =
        new SignalTracker(
            config.signalRates, RangeSensorConfig.CONTROL_FIELDS, RangeSensorTelemetry.class);
    version = signals.track("firmwareVersion", canrange.getVersion());
    isProLicensed = signals.track("isProLicensed", canrange.getIsProLicensed());
    distance = signals.track("distance", canrange.getDistance());
    distanceStdDev = signals.track("distanceStdDev", canrange.getDistanceStdDev());
    measurementTime = signals.track("measurementTime", canrange.getMeasurementTime());
    detected = signals.track("detected", canrange.getIsDetected());
    signalStrength = signals.track("signalStrength", canrange.getSignalStrength());
    ambientSignal = signals.track("ambientSignal", canrange.getAmbientSignal());
    measurementHealth = signals.track("measurementHealth", canrange.getMeasurementHealth());
    fovCenterX = signals.track("fovCenterX", canrange.getRealFOVCenterX());
    fovCenterY = signals.track("fovCenterY", canrange.getRealFOVCenterY());
    fovRangeX = signals.track("fovRangeX", canrange.getRealFOVRangeX());
    fovRangeY = signals.track("fovRangeY", canrange.getRealFOVRangeY());
    supplyVoltage = signals.track("supplyVoltage", canrange.getSupplyVoltage());
    faults = signals.trackFaults(canrange, CANrangeFault.class);
    signals.apply(canrange, bus);
  }

  @Override
  @Trace
  public void updateInputs(RangeSensorTelemetry telemetry) {
    connected = BaseStatusSignal.isAllGood(distance, detected);
    telemetry.connected = connected;
    telemetry.firmwareVersion = version.getValue();
    telemetry.isProLicensed = isProLicensed.getValue();
    telemetry.distance = distance.getValue();
    telemetry.distanceStdDev = distanceStdDev.getValue();
    telemetry.measurementTime = measurementTime.getValue();
    telemetry.detected = detected.getValue();
    telemetry.signalStrength = signalStrength.getValue();
    telemetry.ambientSignal = ambientSignal.getValue();
    telemetry.measurementHealth =
        CANrangeConversions.toMeasurementHealth(measurementHealth.getValue());
    telemetry.fovCenterX = fovCenterX.getValue();
    telemetry.fovCenterY = fovCenterY.getValue();
    telemetry.fovRangeX = fovRangeX.getValue();
    telemetry.fovRangeY = fovRangeY.getValue();
    telemetry.supplyVoltage = supplyVoltage.getValue();
    telemetry.faults = faults.faults();
    telemetry.stickyFaults = faults.stickyFaults();
  }

  @Override
  public boolean isConnected() {
    return connected;
  }

  @Override
  public void clearStickyFaults() {
    canrange.clearStickyFaults();
  }
}
