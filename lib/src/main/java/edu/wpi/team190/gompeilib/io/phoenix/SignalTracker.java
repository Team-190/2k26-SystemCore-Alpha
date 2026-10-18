package edu.wpi.team190.gompeilib.io.phoenix;

import static org.wpilib.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.team190.gompeilib.io.base.HardwareFault;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Collects a Phoenix6 device's status signals, each tagged with the telemetry field it fills, then
 * sets every signal's update rate, trims unused frames, and registers the signals on the device's
 * bus for synchronized refresh.
 */
public final class SignalTracker {
  private final SignalRates rates;
  private final Set<String> controlFields;
  private final Map<Double, List<BaseStatusSignal>> signalsByRate = new LinkedHashMap<>();
  private final List<BaseStatusSignal> signals = new ArrayList<>();

  /**
   * Creates a tracker.
   *
   * @param rates the device's configured rates
   * @param controlFields the device's control-relevant telemetry fields
   * @param telemetry the device's telemetry class, to check {@code rates}' overrides against
   */
  public SignalTracker(SignalRates rates, Set<String> controlFields, Class<?> telemetry) {
    rates.requireFieldsOf(telemetry);
    this.rates = rates;
    this.controlFields = controlFields;
  }

  /**
   * Tracks a signal at the rate of the telemetry field it fills.
   *
   * @param field the telemetry field
   * @param signal the signal
   * @param <T> the signal's value type
   * @return {@code signal}
   */
  public <T> StatusSignal<T> track(String field, StatusSignal<T> signal) {
    double hertz = rates.rateFor(field, controlFields).in(Hertz);
    signalsByRate.computeIfAbsent(hertz, rate -> new ArrayList<>()).add(signal);
    signals.add(signal);
    return signal;
  }

  /**
   * Tracks every live and sticky fault signal on a device.
   *
   * @param device the device
   * @param type the device's fault enum
   * @param <F> the device's fault enum
   * @return the fault signals
   */
  public <F extends Enum<F> & HardwareFault> FaultSignals<F> trackFaults(
      Object device, Class<F> type) {
    return FaultSignals.of(device, type, this::track);
  }

  /**
   * Applies every tracked signal's rate, disables the device's unused frames, and registers the
   * signals for synchronized refresh on {@code bus}.
   *
   * @param device the device
   * @param bus the device's bus
   */
  public void apply(ParentDevice device, CANBus bus) {
    for (Map.Entry<Double, List<BaseStatusSignal>> rate : signalsByRate.entrySet()) {
      BaseStatusSignal.setUpdateFrequencyForAll(
          rate.getKey(), rate.getValue().toArray(BaseStatusSignal[]::new));
    }
    device.optimizeBusUtilization();
    PhoenixUtil.registerSignals(bus, signals.toArray(BaseStatusSignal[]::new));
  }
}
