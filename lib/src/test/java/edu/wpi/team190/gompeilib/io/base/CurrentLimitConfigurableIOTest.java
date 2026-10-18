package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import org.junit.jupiter.api.Test;

public class CurrentLimitConfigurableIOTest {

  @Test
  public void testDefaultMethods() {
    CurrentLimitConfigurableIO io = new CurrentLimitConfigurableIO() {};

    assertDoesNotThrow(() -> io.updateCurrentLimits(new CurrentLimits(40.0, 60.0)));
  }
}
