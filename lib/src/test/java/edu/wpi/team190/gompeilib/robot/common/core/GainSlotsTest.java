package edu.wpi.team190.gompeilib.robot.common.core;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import org.junit.jupiter.api.Test;

public class GainSlotsTest {

  @Test
  public void shouldStartEveryTypeOnSlotZero() {
    GainSlots slots = new GainSlots();
    for (OutputType type : OutputType.values()) {
      assertEquals(GainSlot.ZERO, slots.get(type));
    }
  }

  @Test
  public void shouldSetOneOrEveryType() {
    GainSlots slots = new GainSlots();
    slots.set(OutputType.CURRENT, GainSlot.ONE);
    assertEquals(GainSlot.ONE, slots.get(OutputType.CURRENT));
    assertEquals(GainSlot.ZERO, slots.get(OutputType.VOLTAGE));

    slots.set(GainSlot.TWO);
    for (OutputType type : OutputType.values()) {
      assertEquals(GainSlot.TWO, slots.get(type));
    }
  }

  @Test
  public void shouldRejectNulls() {
    GainSlots slots = new GainSlots();
    assertThrows(NullPointerException.class, () -> slots.set(null));
    assertThrows(NullPointerException.class, () -> slots.set(null, GainSlot.ZERO));
    assertThrows(NullPointerException.class, () -> slots.set(OutputType.VOLTAGE, null));
    assertThrows(NullPointerException.class, () -> slots.get(null));
  }
}
