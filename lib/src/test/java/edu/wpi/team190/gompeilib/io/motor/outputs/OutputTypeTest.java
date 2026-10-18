package edu.wpi.team190.gompeilib.io.motor.outputs;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import java.util.Optional;
import org.junit.jupiter.api.Test;

public class OutputTypeTest {

  @Test
  public void testConstants() {
    IOTestSupport.assertEnumRoundTrip(OutputType.class, "DUTY_CYCLE", "VOLTAGE", "CURRENT");
  }

  @Test
  public void shouldMapUnitsToOutputTypes() {
    assertEquals(Optional.of(OutputType.DUTY_CYCLE), OutputType.of(Percent));
    assertEquals(Optional.of(OutputType.DUTY_CYCLE), OutputType.of(Value));
    assertEquals(Optional.of(OutputType.VOLTAGE), OutputType.of(Volts));
    assertEquals(Optional.of(OutputType.VOLTAGE), OutputType.of(Millivolts));
    assertEquals(Optional.of(OutputType.CURRENT), OutputType.of(Amps));
    assertEquals(Optional.empty(), OutputType.of(MetersPerSecond));
  }
}
