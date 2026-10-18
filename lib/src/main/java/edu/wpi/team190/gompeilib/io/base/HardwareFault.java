package edu.wpi.team190.gompeilib.io.base;

import java.util.EnumSet;
import java.util.Set;

/**
 * A single hardware fault bit within a packed {@code long} fault field.
 *
 * <p>Different vendors number their faults differently, so each vendor device defines its own enum
 * implementing this interface. The bit positions are GompeiLib's own; vendor implementations pack
 * them from the device's individual fault accessors.
 */
public interface HardwareFault {
  /**
   * Returns the bit position of this fault within a packed fault field.
   *
   * @return the bit position, in {@code [0, 64)}
   */
  int bit();

  /**
   * Returns the single-bit mask for this fault.
   *
   * @return {@code 1L << bit()}
   */
  default long mask() {
    return 1L << bit();
  }

  /**
   * Checks whether a packed fault field contains the given fault.
   *
   * @param faults the packed fault field
   * @param fault the fault to check for
   * @param <F> the vendor fault enum
   * @return true if the fault bit is set, false otherwise or if {@code fault} is null
   */
  static <F extends Enum<F> & HardwareFault> boolean hasFault(long faults, F fault) {
    if (fault == null) {
      return false;
    }
    return (faults & fault.mask()) != 0;
  }

  /**
   * Decodes a packed fault field into the set of active faults.
   *
   * @param faults the packed fault field
   * @param type the vendor fault enum class
   * @param <F> the vendor fault enum
   * @return the set of active faults
   */
  static <F extends Enum<F> & HardwareFault> Set<F> activeFaults(long faults, Class<F> type) {
    Set<F> active = EnumSet.noneOf(type);
    for (F fault : type.getEnumConstants()) {
      if (hasFault(faults, fault)) {
        active.add(fault);
      }
    }
    return active;
  }
}
