package edu.wpi.team190.gompeilib.tuning;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.tunable.TunableTable;
import org.wpilib.units.ImmutableMeasure;
import org.wpilib.units.TimeUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Per;

public class TunableMeasureTest extends GompeiLibTestBase {
  private MockTunableBackend backend;

  @Override
  protected boolean isTuningMode() {
    return true;
  }

  @BeforeEach
  public void setUp() {
    TunableRegistry.reset();
    backend = new MockTunableBackend();
    TunableRegistry.registerBackend("", backend);
  }

  @AfterEach
  public void tearDown() {
    TunableRegistry.reset();
  }

  @Test
  public void shouldPublishTheMagnitudeInTheDeclaredUnit() {
    TunableTable table = GompeiTunables.table("Measure");
    TunableMeasure<AngularVelocity> velocity =
        GompeiTunables.measure(table, "Velocity", RotationsPerSecond.of(2.0), null);

    assertEquals(2.0, backend.getDouble("/Measure/Velocity"));
    assertSame(RotationsPerSecond, velocity.unit());
    assertEquals(4.0 * Math.PI, velocity.get().in(RadiansPerSecond), 1e-9);
    assertSame(velocity.getInnerTunable(), backend.getTunable("/Measure/Velocity"));
  }

  @Test
  public void shouldReadEditsInTheDeclaredUnit() {
    AtomicInteger tuned = new AtomicInteger(0);
    TunableTable table = GompeiTunables.table("Measure");
    TunableMeasure<Distance> distance =
        GompeiTunables.measure(table, "Distance", Inches.of(3.0), tuned::incrementAndGet);

    backend.setDouble("/Measure/Distance", 6.0);
    TunableRegistry.update();

    assertEquals(6.0, distance.get().in(Inches), 1e-9);
    assertSame(Inches, distance.get().unit());
    assertEquals(1, tuned.get());
  }

  @Test
  public void shouldConvertSetValuesIntoTheDeclaredUnit() {
    TunableMeasure<Distance> distance =
        GompeiTunables.measure(GompeiTunables.table("Measure"), "Set", Meters.of(1.0), null);

    distance.set(Centimeters.of(50.0));
    assertEquals(0.5, distance.get().in(Meters), 1e-9);
    assertSame(Meters, distance.get().unit());
    assertEquals(0.5, backend.getDouble("/Measure/Set"), 1e-9);
    assertTrue(distance.hasChanged());
    assertTrue(distance.supportsChangeNotification());
    distance.resetChanged();
    assertFalse(distance.hasChanged());
  }

  @Test
  @SuppressWarnings({"unchecked", "rawtypes"})
  public void shouldRejectIncompatibleUnits() {
    TunableMeasure distance =
        GompeiTunables.measure(GompeiTunables.table("Measure"), "Bad", Meters.of(1.0), null);
    assertThrows(IllegalArgumentException.class, () -> distance.set(Seconds.of(1.0)));
  }

  @Test
  public void shouldReportItsMeasureType() {
    TunableMeasure<Distance> distance = GompeiTunables.measure(null, "Type", Meters.of(1.0), null);
    assertTrue(Distance.class.isAssignableFrom(distance.getTypeClass()));
  }

  @Test
  public void shouldKeepAPerMeasureAPerAfterAnEdit() {
    // Volts.per(Second) is a velocity unit, whose of() would build a Velocity, not a Per
    TunableMeasure<Per<VoltageUnit, TimeUnit>> rate =
        GompeiTunables.measure(
            GompeiTunables.table("Measure"), "Rate", VoltsPerSecond.ofNative(1.0), null);

    backend.setDouble("/Measure/Rate", 2.0);
    TunableRegistry.update();

    Per<VoltageUnit, TimeUnit> edited = rate.get();
    assertEquals(2.0, edited.in(VoltsPerSecond), 1e-9);
  }

  @Test
  public void shouldRejectMeasuresItCantRebuild() {
    // A generic ImmutableMeasure's unit rebuilds a Distance, not another ImmutableMeasure
    assertThrows(
        IllegalArgumentException.class,
        () ->
            GompeiTunables.measure(
                null, "Generic", ImmutableMeasure.ofRelativeUnits(1, Meters), null));
  }
}
