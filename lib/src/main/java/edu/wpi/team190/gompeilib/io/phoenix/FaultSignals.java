package edu.wpi.team190.gompeilib.io.phoenix;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.team190.gompeilib.io.base.HardwareFault;
import java.lang.reflect.Method;
import java.util.EnumMap;
import java.util.Locale;
import java.util.Map;
import java.util.function.BiFunction;

/**
 * A Phoenix6 device's individual live and sticky fault signals, packed into {@link HardwareFault}
 * bit fields. Packing from the per-fault getters avoids depending on the device's raw fault-field
 * layout.
 *
 * @param <F> the device's fault enum
 */
public final class FaultSignals<F extends Enum<F> & HardwareFault> {
  private final Map<F, StatusSignal<Boolean>> live;
  private final Map<F, StatusSignal<Boolean>> sticky;

  private FaultSignals(Map<F, StatusSignal<Boolean>> live, Map<F, StatusSignal<Boolean>> sticky) {
    this.live = live;
    this.sticky = sticky;
  }

  /**
   * Finds every {@code getFault_*} and {@code getStickyFault_*} getter on a device and matches each
   * to the fault enum by name.
   *
   * @param device the Phoenix6 device
   * @param type the device's fault enum
   * @param track called with {@code "faults"} or {@code "stickyFaults"} and each signal, returning
   *     the signal to keep
   * @param <F> the device's fault enum
   * @return the fault signals
   * @throws IllegalStateException if a getter has no matching fault, or a fault has no getter
   */
  @SuppressWarnings("unchecked") // Every fault getter returns StatusSignal<Boolean>
  public static <F extends Enum<F> & HardwareFault> FaultSignals<F> of(
      Object device,
      Class<F> type,
      BiFunction<String, StatusSignal<Boolean>, StatusSignal<Boolean>> track) {
    Map<F, StatusSignal<Boolean>> live = new EnumMap<>(type);
    Map<F, StatusSignal<Boolean>> sticky = new EnumMap<>(type);
    for (Method method : device.getClass().getMethods()) {
      boolean isSticky = method.getName().startsWith("getStickyFault_");
      if (method.getParameterCount() != 0
          || !(isSticky || method.getName().startsWith("getFault_"))) {
        continue;
      }
      F fault = named(type, method.getName().substring(method.getName().indexOf('_') + 1));
      StatusSignal<Boolean> signal;
      try {
        signal = (StatusSignal<Boolean>) method.invoke(device);
      } catch (ReflectiveOperationException e) {
        throw new IllegalStateException("Couldn't read " + method.getName(), e);
      }
      (isSticky ? sticky : live)
          .put(fault, track.apply(isSticky ? "stickyFaults" : "faults", signal));
    }
    for (F fault : type.getEnumConstants()) {
      if (!live.containsKey(fault) || !sticky.containsKey(fault)) {
        throw new IllegalStateException(
            device.getClass().getSimpleName() + " has no fault getter for " + fault);
      }
    }
    return new FaultSignals<>(live, sticky);
  }

  private static <F extends Enum<F>> F named(Class<F> type, String phoenixName) {
    String normalized = normalize(phoenixName);
    for (F fault : type.getEnumConstants()) {
      if (normalize(fault.name()).equals(normalized)) {
        return fault;
      }
    }
    throw new IllegalStateException(
        type.getSimpleName() + " has no entry for Phoenix6 fault " + phoenixName);
  }

  /** Enum constants can't start with a digit, so "5V" is spelled out as "FIVE_V". */
  private static String normalize(String name) {
    return name.replace("_", "").toUpperCase(Locale.ROOT).replace("5V", "FIVEV");
  }

  /**
   * Returns the live faults, packed.
   *
   * @return the live fault bits
   */
  public long faults() {
    return pack(live);
  }

  /**
   * Returns the sticky faults, packed.
   *
   * @return the sticky fault bits
   */
  public long stickyFaults() {
    return pack(sticky);
  }

  private long pack(Map<F, StatusSignal<Boolean>> signals) {
    long packed = 0L;
    for (Map.Entry<F, StatusSignal<Boolean>> fault : signals.entrySet()) {
      if (Boolean.TRUE.equals(fault.getValue().getValue())) {
        packed |= fault.getKey().mask();
      }
    }
    return packed;
  }
}
