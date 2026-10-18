package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.wpilib.units.Units.Rotations;

import org.junit.jupiter.api.Test;

public class PositionResettableIOTest {

  @Test
  public void testDefaultMethods() {
    PositionResettableIO io = new PositionResettableIO() {};

    assertDoesNotThrow(() -> io.setPosition(Rotations.of(1.5)));
  }
}
