package edu.wpi.team190.gompeilib.control.constraints;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.units.*;

public class AngularPositionConstraintsTest extends GompeiLibTestBase {
  @Override
  protected boolean isTuningMode() {
    return true;
  }

  @Test
  public void testAngularPositionConstraints() {
    AngularPositionConstraints c =
        AngularPositionConstraints.builder()
            .withPrefix("Test")
            .withGoalTolerance(Units.Degrees.of(1.0))
            .withMaxVelocity(Units.DegreesPerSecond.of(180.0))
            .withMaxAcceleration(Units.DegreesPerSecond.per(Units.Second).of(360.0))
            .build();

    assertEquals(1.0, c.getGoalTolerance(Units.Degrees));
    assertEquals(180.0, c.getMaxVelocity(Units.DegreesPerSecond));
    assertEquals(360.0, c.getMaxAcceleration(Units.DegreesPerSecond.per(Units.Second)));

    c.onChange(l -> {});
    assertNotNull(AngularPositionConstraints.builder().toString());

    try {
      var f = AngularPositionConstraints.class.getDeclaredField("lastNotifiedValues");
      f.setAccessible(true);
      f.set(c, null);
      var notify = AngularPositionConstraints.class.getDeclaredMethod("notifyListeners");
      notify.setAccessible(true);
      notify.invoke(c);
      notify.invoke(c);
    } catch (Exception e) {
      fail(e);
    }

    // No prefix means unpublished; unset values default to zero
    AngularPositionConstraints defaults = AngularPositionConstraints.builder().build();
    assertEquals(0.0, defaults.goalTolerance().get().baseUnitMagnitude());
    assertEquals(0.0, defaults.maxVelocity().get().baseUnitMagnitude());
    assertEquals(0.0, defaults.maxAcceleration().get().baseUnitMagnitude());
  }

  @Test
  public void testCharacterizationMarginDefaultsToTheGoalTolerance() {
    TunableRegistry.reset();
    MockTunableBackend backend = new MockTunableBackend();
    TunableRegistry.registerBackend("", backend);
    AngularPositionConstraints defaults =
        AngularPositionConstraints.builder().withGoalTolerance(Units.Radians.of(0.05)).build();
    assertEquals(0.05, defaults.characterizationMargin().get().in(Units.Radians), 1e-9);

    AngularPositionConstraints c =
        AngularPositionConstraints.builder()
            .withPrefix("Margin")
            .withGoalTolerance(Units.Radians.of(0.05))
            .withCharacterizationMargin(Units.Radians.of(0.2))
            .build();
    assertEquals(0.2, c.characterizationMargin().get().in(Units.Radians), 1e-9);

    // Only characterization reads the margin, so editing it leaves the profile listeners alone.
    AtomicInteger counter = new AtomicInteger(0);
    c.onChange(x -> counter.incrementAndGet());
    backend.setDouble("/Margin/Characterization Margin", 0.3);
    TunableRegistry.update();
    assertEquals(0.3, c.characterizationMargin().get().in(Units.Radians), 1e-9);
    assertEquals(0, counter.get());
    TunableRegistry.reset();
  }

  @Test
  public void shouldPublishAndEditInTheDeclaredUnit() {
    TunableRegistry.reset();
    MockTunableBackend backend = new MockTunableBackend();
    TunableRegistry.registerBackend("", backend);
    AngularPositionConstraints c =
        AngularPositionConstraints.builder()
            .withPrefix("Hood")
            .withGoalTolerance(Units.Degrees.of(1.0))
            .withMaxVelocity(Units.RotationsPerSecond.of(2.0))
            .build();

    // Shown as the Motion Magic number from Phoenix Tuner, not its radians-per-second base value
    assertEquals(2.0, backend.getDouble("/Hood/Max Velocity"));
    assertEquals(1.0, backend.getDouble("/Hood/Goal Tolerance"));
    assertEquals(
        Units.RotationsPerSecond.symbol(),
        backend.getValue("/Hood/Max Velocity" + GompeiTunables.UNIT_SUFFIX, String.class));
    assertEquals(
        Units.Degrees.symbol(),
        backend.getValue("/Hood/Goal Tolerance" + GompeiTunables.UNIT_SUFFIX, String.class));
    // Unset constraints publish zero in rotations
    assertEquals(0.0, backend.getDouble("/Hood/Max Acceleration"));
    assertEquals(
        Units.RotationsPerSecondPerSecond.symbol(),
        backend.getValue("/Hood/Max Acceleration" + GompeiTunables.UNIT_SUFFIX, String.class));

    backend.setDouble("/Hood/Max Velocity", 3.0);
    TunableRegistry.update();
    assertEquals(3.0, c.getMaxVelocity(Units.RotationsPerSecond), 1e-9);
    assertEquals(6.0 * Math.PI, c.getMaxVelocity(Units.RadiansPerSecond), 1e-9);
    TunableRegistry.reset();
  }
}
