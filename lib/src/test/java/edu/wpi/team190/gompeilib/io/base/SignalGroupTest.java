package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import edu.wpi.team190.gompeilib.io.base.SignalRatesTest.Groups;
import edu.wpi.team190.gompeilib.io.base.SignalRatesTest.Signals;
import org.junit.jupiter.api.Test;

public class SignalGroupTest {

  @Test
  public void testDefaultRateSeedsTheDefaults() {
    SignalRates<Signals, Groups> rates = SignalRates.defaults(Signals.class, Groups.class);

    for (Groups group : Groups.values()) {
      assertEquals(group.defaultRate(), rates.rateOf(group));
    }
    assertEquals(Hertz.of(100.0), Groups.MOTION.defaultRate());
  }
}
