package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.wpilib.units.Units.Degrees;

import org.junit.jupiter.api.Test;

public class YawResettableIOTest {

  @Test
  public void testDefaultMethods() {
    YawResettableIO io = new YawResettableIO() {};

    assertDoesNotThrow(() -> io.setYaw(Degrees.of(90.0)));
  }
}
