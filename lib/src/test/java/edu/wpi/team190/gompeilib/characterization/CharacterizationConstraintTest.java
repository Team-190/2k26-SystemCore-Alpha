package edu.wpi.team190.gompeilib.characterization;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.measure.Distance;

public class CharacterizationConstraintTest {
  private final AtomicReference<Distance> position = new AtomicReference<>(Meters.of(0.5));
  private final AtomicReference<Distance> margin = new AtomicReference<>(Meters.of(0.1));
  private final CharacterizationConstraint<DistanceUnit> constraint =
      new CharacterizationConstraint<>(position::get, Meters.of(0), Meters.of(1), margin::get);

  @Test
  public void testAwayFromBothLimits() {
    assertFalse(constraint.atLimit(true));
    assertFalse(constraint.atLimit(false));
  }

  @Test
  public void testUpperLimitIsOneSided() {
    position.set(Meters.of(0.9));
    assertTrue(constraint.atLimit(true));
    assertFalse(constraint.atLimit(false));

    position.set(Meters.of(1.2));
    assertTrue(constraint.atLimit(true));

    position.set(Meters.of(0.89));
    assertFalse(constraint.atLimit(true));
  }

  @Test
  public void testLowerLimitIsOneSided() {
    position.set(Meters.of(0.1));
    assertTrue(constraint.atLimit(false));
    assertFalse(constraint.atLimit(true));

    position.set(Meters.of(-0.2));
    assertTrue(constraint.atLimit(false));

    position.set(Meters.of(0.11));
    assertFalse(constraint.atLimit(false));
  }

  @Test
  public void testComparesAcrossUnitsAndReadsTheMarginEachTime() {
    position.set(Centimeters.of(95));
    assertTrue(constraint.atLimit(true));
    margin.set(Millimeters.of(0));
    assertFalse(constraint.atLimit(true));

    position.set(Centimeters.of(0));
    assertTrue(constraint.atLimit(false));
    position.set(Centimeters.of(-1));
    assertTrue(constraint.atLimit(false));
  }

  @Test
  public void shouldRequireEveryArgument() {
    assertThrows(
        NullPointerException.class,
        () -> new CharacterizationConstraint<>(null, Meters.of(0), Meters.of(1), margin::get));
    assertThrows(
        NullPointerException.class,
        () -> new CharacterizationConstraint<>(position::get, null, Meters.of(1), margin::get));
    assertThrows(
        NullPointerException.class,
        () -> new CharacterizationConstraint<>(position::get, Meters.of(0), null, margin::get));
    assertThrows(
        NullPointerException.class,
        () ->
            new CharacterizationConstraint<DistanceUnit>(
                position::get, Meters.of(0), Meters.of(1), null));
  }
}
