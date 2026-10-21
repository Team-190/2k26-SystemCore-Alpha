package edu.wpi.team190.gompeilib.io.physics;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.KilogramSquareMeters;

import org.junit.jupiter.api.Test;

public class InertiaTest {
  private static final double EPSILON = 1e-12;

  @Test
  public void testAPartAtTheOutputAddsItsOwnInertia() {
    Inertia drum = Inertia.atOutput(KilogramSquareMeters.of(0.09));

    assertEquals(KilogramSquareMeters.of(0.09), drum.momentOfInertia());
    assertEquals(1.0, drum.speedRatio());
    assertEquals(0.09, drum.reflected().in(KilogramSquareMeters), EPSILON);
  }

  @Test
  public void testAGearedPartAddsItsInertiaTimesTheSpeedRatioSquared() {
    Inertia disk = Inertia.geared(KilogramSquareMeters.of(0.01), 3.0);
    assertEquals(3.0, disk.speedRatio());
    assertEquals(0.09, disk.reflected().in(KilogramSquareMeters), EPSILON);

    // Direction doesn't matter: a part spinning backward stores the same energy
    Inertia reversed = Inertia.geared(KilogramSquareMeters.of(0.01), -3.0);
    assertEquals(0.09, reversed.reflected().in(KilogramSquareMeters), EPSILON);
  }

  @Test
  public void testTotalSumsEveryPart() {
    // A drum, a disk geared 2:1 up from it, and two rotors through a 5:3 reduction
    double rotor = 2 * 0.00005;
    double expected = 0.09 + 0.01 * 4 + rotor * (5.0 / 3.0) * (5.0 / 3.0);
    assertEquals(
        expected,
        Inertia.total(
                Inertia.atOutput(KilogramSquareMeters.of(0.09)),
                Inertia.geared(KilogramSquareMeters.of(0.01), 2.0),
                Inertia.geared(KilogramSquareMeters.of(rotor), 5.0 / 3.0))
            .in(KilogramSquareMeters),
        EPSILON);
    assertEquals(0.0, Inertia.total().in(KilogramSquareMeters));
  }

  @Test
  public void shouldRejectPartsThatCantExist() {
    assertThrows(NullPointerException.class, () -> Inertia.atOutput(null));
    assertThrows(
        IllegalArgumentException.class, () -> Inertia.atOutput(KilogramSquareMeters.of(-0.1)));
    assertThrows(
        IllegalArgumentException.class,
        () -> Inertia.geared(KilogramSquareMeters.of(0.1), Double.NaN));
    assertThrows(
        IllegalArgumentException.class,
        () -> Inertia.geared(KilogramSquareMeters.of(0.1), Double.POSITIVE_INFINITY));
  }
}
