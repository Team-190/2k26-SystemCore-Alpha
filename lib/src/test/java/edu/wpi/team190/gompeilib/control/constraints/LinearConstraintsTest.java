package edu.wpi.team190.gompeilib.control.constraints;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.units.*;

public class LinearConstraintsTest extends GompeiLibTestBase {
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
  public void testLinearConstraints() {
    LinearConstraints c =
        LinearConstraints.builder()
            .withPrefix("Test")
            .withGoalTolerance(Units.Meters.of(0.05))
            .withMaxVelocity(Units.MetersPerSecond.of(3.0))
            .withMaxAcceleration(Units.MetersPerSecond.per(Units.Second).of(6.0))
            .build();

    assertEquals(0.05, c.getGoalTolerance(Units.Meters));
    assertEquals(3.0, c.getMaxVelocity(Units.MetersPerSecond));
    assertEquals(6.0, c.getMaxAcceleration(Units.MetersPerSecond.per(Units.Second)));

    // No prefix means unpublished; unset values default to zero
    LinearConstraints defaults = LinearConstraints.builder().build();
    assertEquals(0.0, defaults.goalTolerance().get().in(Units.Meters));
    assertEquals(0.0, defaults.maxVelocity().get().in(Units.MetersPerSecond));
    assertEquals(0.0, defaults.maxAcceleration().get().in(Units.MetersPerSecondPerSecond));
  }

  @Test
  public void testRemoteEditNotifies() {
    LinearConstraints c =
        LinearConstraints.builder()
            .withPrefix("Tuned")
            .withGoalTolerance(Units.Meters.of(0.05))
            .withMaxVelocity(Units.MetersPerSecond.of(3.0))
            .build();
    AtomicInteger counter = new AtomicInteger(0);
    c.onChange(x -> counter.incrementAndGet());

    backend.setDouble("/Tuned/Goal Tolerance", 0.1);
    TunableRegistry.update();

    assertEquals(1, counter.get());
    assertEquals(0.1, c.goalTolerance().get().in(Units.Meters), 1e-9);

    // Update with same value to test deduplication
    backend.setDouble("/Tuned/Goal Tolerance", 0.1);
    TunableRegistry.update();
    assertEquals(1, counter.get());

    assertNotNull(LinearConstraints.builder().toString());
  }

  @Test
  public void shouldPublishAndEditInTheDeclaredUnit() {
    LinearConstraints c =
        LinearConstraints.builder()
            .withPrefix("Declared")
            .withGoalTolerance(Units.Centimeters.of(2.0))
            .withMaxVelocity(Units.FeetPerSecond.of(10.0))
            .build();

    assertEquals(2.0, backend.getDouble("/Declared/Goal Tolerance"));
    assertEquals(10.0, backend.getDouble("/Declared/Max Velocity"));
    assertUnit("/Declared/Goal Tolerance", Units.Centimeters);
    assertUnit("/Declared/Max Velocity", Units.FeetPerSecond);
    // Unset constraints publish zero in meters
    assertEquals(0.0, backend.getDouble("/Declared/Max Acceleration"));
    assertUnit("/Declared/Max Acceleration", Units.MetersPerSecondPerSecond);

    backend.setDouble("/Declared/Goal Tolerance", 5.0);
    TunableRegistry.update();
    assertEquals(0.05, c.getGoalTolerance(Units.Meters), 1e-9);
    assertEquals(5.0, c.getGoalTolerance(Units.Centimeters), 1e-9);
  }

  private void assertUnit(String path, Unit unit) {
    assertEquals(
        unit.symbol(), backend.getValue(path + GompeiTunables.UNIT_SUFFIX, String.class), path);
  }

  @Test
  public void testSysIdMarginDefaultsToTheGoalTolerance() {
    LinearConstraints defaults =
        LinearConstraints.builder().withGoalTolerance(Units.Meters.of(0.05)).build();
    assertEquals(0.05, defaults.sysIdMargin().get().in(Units.Meters), 1e-9);

    LinearConstraints c =
        LinearConstraints.builder()
            .withPrefix("Margin")
            .withGoalTolerance(Units.Meters.of(0.05))
            .withSysIdMargin(Units.Meters.of(0.2))
            .build();
    assertEquals(0.2, c.sysIdMargin().get().in(Units.Meters), 1e-9);

    // Only SysId reads the margin, so editing it leaves the profile listeners alone.
    AtomicInteger counter = new AtomicInteger(0);
    c.onChange(x -> counter.incrementAndGet());
    backend.setDouble("/Margin/SysId Margin", 0.3);
    TunableRegistry.update();
    assertEquals(0.3, c.sysIdMargin().get().in(Units.Meters), 1e-9);
    assertEquals(0, counter.get());
  }
}
