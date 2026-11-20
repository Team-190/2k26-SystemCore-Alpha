package edu.wpi.team190.gompeilib.io.phoenix;

import static org.wpilib.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.team190.gompeilib.io.base.HardwareFault;
import edu.wpi.team190.gompeilib.io.base.Signal;
import edu.wpi.team190.gompeilib.io.base.SignalGroup;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Collects a Phoenix6 device's status signals, each tagged with the {@link Signal} it fills, then
 * sets every signal's update rate, trims unused frames, and registers the signals on the device's
 * bus for synchronized refresh.
 *
 * @param <S> the device's signal enum
 * @param <G> the device's group enum
 */
public final class SignalTracker<S extends Enum<S> & Signal<G>, G extends Enum<G> & SignalGroup> {
  private final SignalRates<S, G> rates;
  private final Map<Double, List<BaseStatusSignal>> signalsByRate = new LinkedHashMap<>();
  private final List<BaseStatusSignal> signals = new ArrayList<>();

  /**
   * Creates a tracker.
   *
   * @param rates the device's configured rates
   */
  public SignalTracker(SignalRates<S, G> rates) {
    this.rates = rates;
  }

  /**
   * Tracks a status signal at its {@link Signal}'s rate. A signal whose rate is zero is turned off
   * and left out of the synchronized refresh, so it holds its last value.
   *
   * @param signal which signal this is
   * @param status the status signal
   * @param <T> the signal's value type
   * @return {@code status}
   */
  public <T> StatusSignal<T> track(S signal, StatusSignal<T> status) {
    double hertz = rates.rateFor(signal).in(Hertz);
    signalsByRate.computeIfAbsent(hertz, rate -> new ArrayList<>()).add(status);
    if (hertz > 0.0) {
      signals.add(status);
    }
    return status;
  }

  /**
   * Tracks every live and sticky fault signal on a device.
   *
   * @param device the device
   * @param type the device's fault enum
   * @param live the signal the live faults fill
   * @param sticky the signal the sticky faults fill
   * @param <F> the device's fault enum
   * @return the fault signals
   */
  public <F extends Enum<F> & HardwareFault> FaultSignals<F> trackFaults(
      Object device, Class<F> type, S live, S sticky) {
    return FaultSignals.of(
        device, type, status -> track(live, status), status -> track(sticky, status));
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
