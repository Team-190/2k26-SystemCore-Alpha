package edu.wpi.team190.gompeilib.subsystems.elevator;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionState;
import org.junit.jupiter.api.Test;

public class ElevatorStateTest {
  @Test
  public void testEnum() {
    assertEquals(3, ExtensionState.values().length);
    assertEquals(ExtensionState.IDLE, ExtensionState.valueOf("IDLE"));
    assertEquals(
        ExtensionState.OPEN_LOOP_VOLTAGE_CONTROL,
        ExtensionState.valueOf("OPEN_LOOP_VOLTAGE_CONTROL"));
    assertEquals(
        ExtensionState.CLOSED_LOOP_POSITION_CONTROL,
        ExtensionState.valueOf("CLOSED_LOOP_POSITION_CONTROL"));
  }
}
