package edu.wpi.team190.gompeilib.core.io.motor.request;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class DifferentialTest {

  private static final AxisRequest VOLTAGE = new OpenLoopRequest.VoltageOutput(Volts.of(2.0));
  private static final AxisRequest POSITION_VOLTAGE =
      new PositionRequest.PositionVoltage(Rotations.of(1.0));

  @Test
  public void testCanonicalConstructorStoresEveryComponent() throws ReflectiveOperationException {
    IOTestSupport.assertCanonicalConstructors(Differential.class);
  }

  @Test
  public void testShortConstructorUsesDefaults() throws ReflectiveOperationException {
    IOTestSupport.assertShortConstructors(Differential.class, IOTestSupport.MOTOR_REQUEST_DEFAULTS);
  }

  @Test
  public void testAcceptsOpenAverageWithClosedDifference() {
    Differential request = new Differential(VOLTAGE, POSITION_VOLTAGE);

    assertEquals(VOLTAGE, request.average());
    assertEquals(POSITION_VOLTAGE, request.difference());
  }

  @Test
  public void shouldRejectDynamicProfileAverage() {
    AxisRequest dynamic =
        new PositionRequest.DynamicProfiledPositionVoltage(
            Rotations.of(1.0),
            RotationsPerSecond.of(1.0),
            RotationsPerSecondPerSecond.of(1.0),
            RotationsPerSecondPerSecond.per(Second).of(1.0));

    assertThrows(IllegalArgumentException.class, () -> new Differential(dynamic, VOLTAGE));
  }

  @Test
  public void shouldRejectProfiledDifference() {
    AxisRequest profiled = new PositionRequest.ProfiledPositionVoltage(Rotations.of(1.0));

    assertThrows(IllegalArgumentException.class, () -> new Differential(VOLTAGE, profiled));
  }

  @Test
  public void shouldRejectMismatchedOutputTypes() {
    AxisRequest current = new OpenLoopRequest.CurrentOutput(Amps.of(1.0));

    assertThrows(IllegalArgumentException.class, () -> new Differential(VOLTAGE, current));
  }

  @Test
  public void shouldRejectNulls() {
    assertThrows(NullPointerException.class, () -> new Differential(null, VOLTAGE));
    assertThrows(NullPointerException.class, () -> new Differential(VOLTAGE, null));
    assertThrows(
        NullPointerException.class, () -> new Differential(VOLTAGE, POSITION_VOLTAGE, null));
  }

  @Test
  public void testValidityPredicates() {
    assertTrue(Differential.isValidAverage(POSITION_VOLTAGE));
    assertTrue(Differential.isValidDifference(VOLTAGE));
    assertTrue(Differential.isValidDifference(POSITION_VOLTAGE));
    assertFalse(
        Differential.isValidDifference(
            new VelocityRequest.ProfiledVelocityVoltage(RotationsPerSecond.of(1.0))));
  }
}
