package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import org.junit.jupiter.api.Test;

public class StickyFaultClearableIOTest {

  @Test
  public void testDefaultMethods() {
    StickyFaultClearableIO io = new StickyFaultClearableIO() {};

    assertDoesNotThrow(io::clearStickyFaults);
  }
}
