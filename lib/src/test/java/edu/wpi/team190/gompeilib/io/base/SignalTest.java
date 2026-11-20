package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import edu.wpi.team190.gompeilib.io.base.SignalRatesTest.Groups;
import edu.wpi.team190.gompeilib.io.base.SignalRatesTest.Signals;
import org.junit.jupiter.api.Test;

public class SignalTest {

  @Test
  public void testASignalFollowsItsGroupsRate() {
    SignalRates<Signals, Groups> rates =
        SignalRates.defaults(Signals.class, Groups.class)
            .withGroup(Signals.VELOCITY.group(), Hertz.of(20.0));

    assertEquals(Groups.MOTION, Signals.VELOCITY.group());
    assertEquals(Hertz.of(20.0), rates.rateFor(Signals.POSITION));
  }
}
