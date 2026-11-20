package edu.wpi.team190.gompeilib.robot.common.core;

import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;

/**
 * The gain slot a closed-loop control uses for each output type. Every type starts on {@link
 * GainSlot#ZERO}; a mechanism whose voltage and current gains differ gives each type its own slot.
 */
public final class GainSlots {
  private final Map<OutputType, GainSlot> slots = new EnumMap<>(OutputType.class);

  /** Creates slots with every output type on {@link GainSlot#ZERO}. */
  public GainSlots() {
    set(GainSlot.ZERO);
  }

  /**
   * Uses one slot for every output type.
   *
   * @param slot the slot
   */
  public void set(GainSlot slot) {
    Objects.requireNonNull(slot, "slot");
    for (OutputType type : OutputType.values()) {
      slots.put(type, slot);
    }
  }

  /**
   * Uses a slot for one output type.
   *
   * @param type the output type
   * @param slot the slot
   */
  public void set(OutputType type, GainSlot slot) {
    slots.put(Objects.requireNonNull(type, "type"), Objects.requireNonNull(slot, "slot"));
  }

  /**
   * Returns the slot an output type uses.
   *
   * @param type the output type
   * @return the slot
   */
  public GainSlot get(OutputType type) {
    return slots.get(Objects.requireNonNull(type, "type"));
  }
}
