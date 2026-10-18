package edu.wpi.team190.gompeilib.control;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.robot.RobotMode;
import edu.wpi.team190.gompeilib.tuning.GompeiTunables;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.MultUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;

public class GainsTest extends GompeiLibTestBase {
  private static final double EPSILON = 1e-9;
  private static final List<String> TERMS = List.of("kP", "kI", "kD", "kS", "kV", "kA", "kG");

  private MockTunableBackend backend;

  private void configureTuning(boolean isTuning) {
    if (isTuning) {
      reinitGompeiLib(RobotMode.SIM, true, 0.02);
    }
    TunableRegistry.reset();
    backend = new MockTunableBackend();
    TunableRegistry.registerBackend("", backend);
  }

  @AfterEach
  public void tearDown() {
    TunableRegistry.reset();
  }

  /** Hood-style gains, tuned per rotation in Phoenix Tuner. */
  private static Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> perRotation(String prefix) {
    return Gains.angularPosition(Volts)
        .withPrefix(prefix)
        .withKP(Volts.per(Rotations).ofNative(1.0))
        .withKI(Volts.per(MultUnit.combine(Rotations, Seconds)).ofNative(2.0))
        .withKD(Volts.per(RotationsPerSecond).ofNative(3.0))
        .withKS(Volts.of(4.0))
        .withKV(Volts.per(RotationsPerSecond).ofNative(5.0))
        .withKA(Volts.per(RotationsPerSecondPerSecond).ofNative(6.0))
        .withKG(Volts.of(7.0))
        .build();
  }

  @Test
  public void shouldReadBackInTheDeclaredUnit() {
    configureTuning(true);
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> gains = perRotation("Test");

    assertEquals(1.0, gains.getKP(Volts.per(Rotations)), EPSILON);
    assertEquals(2.0, gains.getKI(Volts.per(MultUnit.combine(Rotations, Seconds))), EPSILON);
    assertEquals(3.0, gains.getKD(Volts.per(RotationsPerSecond)), EPSILON);
    assertEquals(4.0, gains.getKS(Volts), EPSILON);
    assertEquals(5.0, gains.getKV(Volts.per(RotationsPerSecond)), EPSILON);
    assertEquals(6.0, gains.getKA(Volts.per(RotationsPerSecondPerSecond)), EPSILON);
    assertEquals(7.0, gains.getKG(Volts), EPSILON);
  }

  @Test
  public void shouldConvertEveryGainBetweenRotationsAndRadians() {
    configureTuning(false);
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> gains = perRotation(null);
    double radiansPerRotation = 2.0 * Math.PI;

    assertEquals(1.0 / radiansPerRotation, gains.getKP(Volts.per(Radians)), EPSILON);
    assertEquals(
        2.0 / radiansPerRotation,
        gains.getKI(Volts.per(MultUnit.combine(Radians, Seconds))),
        EPSILON);
    assertEquals(3.0 / radiansPerRotation, gains.getKD(Volts.per(RadiansPerSecond)), EPSILON);
    assertEquals(5.0 / radiansPerRotation, gains.getKV(Volts.per(RadiansPerSecond)), EPSILON);
    assertEquals(
        6.0 / radiansPerRotation, gains.getKA(Volts.per(RadiansPerSecondPerSecond)), EPSILON);
    // kS and kG are plain output, so the controlled unit doesn't scale them
    assertEquals(4.0, gains.getKS(Volts), EPSILON);
    assertEquals(7.0, gains.getKG(Volts), EPSILON);
  }

  @Test
  public void shouldConvertEveryGainBetweenOutputUnits() {
    configureTuning(false);
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> gains = perRotation(null);

    assertEquals(1000.0, gains.getKP(Millivolts.per(Rotations)), 1e-6);
    assertEquals(2000.0, gains.getKI(Millivolts.per(MultUnit.combine(Rotations, Seconds))), 1e-6);
    assertEquals(3000.0, gains.getKD(Millivolts.per(RotationsPerSecond)), 1e-6);
    assertEquals(4000.0, gains.getKS(Millivolts), 1e-6);
    assertEquals(5000.0, gains.getKV(Millivolts.per(RotationsPerSecond)), 1e-6);
    assertEquals(6000.0, gains.getKA(Millivolts.per(RotationsPerSecondPerSecond)), 1e-6);
    assertEquals(7000.0, gains.getKG(Millivolts), 1e-6);
  }

  @Test
  public void shouldConvertLinearGains() {
    configureTuning(false);
    Gains<VoltageUnit, DistanceUnit, LinearVelocityUnit> gains =
        Gains.linearPosition(Volts)
            .withKP(Volts.per(Meters).ofNative(10.0))
            .withKV(Volts.per(MetersPerSecond).ofNative(2.0))
            .build();

    assertEquals(0.1, gains.getKP(Volts.per(Centimeters)), EPSILON);
    assertEquals(0.02, gains.getKV(Volts.per(Centimeters.per(Second))), EPSILON);
  }

  @Test
  public void shouldDefaultUnsetGainsToZeroInTheDefaultUnit() {
    configureTuning(true);
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> gains =
        Gains.angularPosition(Volts).withPrefix("Unset").build();

    assertEquals(0.0, gains.getKP(Volts.per(Radians)));
    assertEquals(0.0, gains.getKI(Volts.per(MultUnit.combine(Radians, Seconds))));
    assertEquals(0.0, gains.getKD(Volts.per(RadiansPerSecond)));
    assertEquals(0.0, gains.getKS(Volts));
    assertEquals(0.0, gains.getKV(Volts.per(RadiansPerSecond)));
    assertEquals(0.0, gains.getKA(Volts.per(RadiansPerSecondPerSecond)));
    assertEquals(0.0, gains.getKG(Volts));

    // Unset gains are published in the builder's default unit: volts per rotation
    assertUnit("/Unset/kP", Volts.per(Rotations));
    assertUnit("/Unset/kI", Volts.per(MultUnit.combine(Rotations, Seconds)));
    assertUnit("/Unset/kD", Volts.per(Rotations.per(Second)));
    assertUnit("/Unset/kS", Volts);
    assertUnit("/Unset/kV", Volts.per(RotationsPerSecond));
    assertUnit("/Unset/kA", Volts.per(RotationsPerSecond.per(Second)));
    assertUnit("/Unset/kG", Volts);
    for (String term : TERMS) {
      assertEquals(0.0, backend.getDouble("/Unset/" + term));
    }
  }

  @Test
  public void shouldPublishEachGainInItsDeclaredUnit() {
    configureTuning(true);
    perRotation("Hood");

    // The number shown is the one tuned in Phoenix Tuner, not a per-radian conversion
    assertEquals(1.0, backend.getDouble("/Hood/kP"));
    assertEquals(3.0, backend.getDouble("/Hood/kD"));
    assertEquals(5.0, backend.getDouble("/Hood/kV"));
    assertUnit("/Hood/kP", Volts.per(Rotations));
    assertUnit("/Hood/kI", Volts.per(MultUnit.combine(Rotations, Seconds)));
    assertUnit("/Hood/kD", Volts.per(RotationsPerSecond));
    assertUnit("/Hood/kS", Volts);
    assertUnit("/Hood/kV", Volts.per(RotationsPerSecond));
    assertUnit("/Hood/kA", Volts.per(RotationsPerSecondPerSecond));
    assertUnit("/Hood/kG", Volts);
  }

  @Test
  public void shouldPublishMixedDeclaredUnits() {
    configureTuning(true);
    Gains.angularPosition(Volts)
        .withPrefix("Mixed")
        .withKP(Millivolts.per(Degrees).ofNative(5.0))
        .withKV(Volts.per(RadiansPerSecond).ofNative(0.5))
        .build();

    assertEquals(5.0, backend.getDouble("/Mixed/kP"));
    assertEquals(0.5, backend.getDouble("/Mixed/kV"));
    assertUnit("/Mixed/kP", Millivolts.per(Degrees));
    assertUnit("/Mixed/kV", Volts.per(RadiansPerSecond));
    assertEquals("mV/°", backend.getValue("/Mixed/kP" + GompeiTunables.UNIT_SUFFIX, String.class));
  }

  @Test
  public void shouldReadDashboardEditsInTheDeclaredUnit() {
    configureTuning(true);
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> gains = perRotation("Edit");

    backend.setDouble("/Edit/kP", 2.0);
    backend.setDouble("/Edit/kA", 1.0);
    TunableRegistry.update();

    assertEquals(2.0, gains.getKP(Volts.per(Rotations)), EPSILON);
    assertEquals(2.0 / (2.0 * Math.PI), gains.getKP(Volts.per(Radians)), EPSILON);
    assertEquals(1.0, gains.getKA(Volts.per(RotationsPerSecondPerSecond)), EPSILON);
  }

  @Test
  public void shouldViewGainsInOtherControlledUnits() {
    configureTuning(true);
    Gains<VoltageUnit, DistanceUnit, LinearVelocityUnit> perMeter =
        Gains.linearPosition(Volts)
            .withPrefix("Drum")
            .withKP(Volts.per(Meters).ofNative(10.0))
            .withKS(Volts.of(0.5))
            .withKV(Volts.per(MetersPerSecond).ofNative(4.0))
            .build();
    // A drum of radius 0.05 m: one radian moves 0.05 m
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> perRadian =
        perMeter.view(Radians, RadiansPerSecond, 0.05);

    assertEquals(0.5, perRadian.getKP(Volts.per(Radians)), EPSILON);
    assertEquals(0.5, perRadian.getKS(Volts), EPSILON);
    assertEquals(0.2, perRadian.getKV(Volts.per(RadiansPerSecond)), EPSILON);
    assertSame(Radians, perRadian.controlledUnit());
    assertSame(RadiansPerSecond, perRadian.derivativeUnit());
    assertSame(Volts, perRadian.outputUnit());

    // The view follows edits to the declared gains and notifies with itself
    AtomicInteger counter = new AtomicInteger(0);
    perRadian.onChange(
        g -> {
          assertSame(perRadian, g);
          counter.incrementAndGet();
        });
    backend.setDouble("/Drum/kP", 20.0);
    TunableRegistry.update();
    assertEquals(1, counter.get());
    assertEquals(1.0, perRadian.getKP(Volts.per(Radians)), EPSILON);
  }

  @Test
  public void shouldRejectBadViewRatios() {
    configureTuning(false);
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> gains = perRotation(null);
    assertThrows(IllegalArgumentException.class, () -> gains.view(Radians, RadiansPerSecond, 0.0));
    assertThrows(IllegalArgumentException.class, () -> gains.view(Radians, RadiansPerSecond, -1.0));
    assertThrows(
        IllegalArgumentException.class,
        () -> gains.view(Radians, RadiansPerSecond, Double.POSITIVE_INFINITY));
    assertThrows(
        IllegalArgumentException.class, () -> gains.view(Radians, RadiansPerSecond, Double.NaN));
  }

  @Test
  public void shouldExposeDefaultUnits() {
    configureTuning(false);
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> position =
        Gains.angularPosition(Volts).build();
    assertSame(Volts, position.outputUnit());
    assertSame(Rotations, position.controlledUnit());
    assertSame(RotationsPerSecond, position.derivativeUnit());
    assertSame(Volts, Gains.angularPosition(Millivolts).build().baseOutputUnit());

    var velocity = Gains.angularVelocity(Amps).build();
    assertSame(RotationsPerSecond, velocity.controlledUnit());
    var linear = Gains.linearPosition(Volts).build();
    assertSame(Meters, linear.controlledUnit());
    assertSame(MetersPerSecond, linear.derivativeUnit());
    var linearVelocity = Gains.linearVelocity(Volts).build();
    assertSame(MetersPerSecond, linearVelocity.controlledUnit());
  }

  @Test
  public void shouldRejectNullUnitsAndGains() {
    assertThrows(
        NullPointerException.class, () -> Gains.builder(null, Rotations, RotationsPerSecond));
    assertThrows(NullPointerException.class, () -> Gains.builder(Volts, null, RotationsPerSecond));
    assertThrows(NullPointerException.class, () -> Gains.builder(Volts, Rotations, null));
    assertThrows(NullPointerException.class, () -> Gains.angularPosition(Volts).withKP(null));
  }

  @Test
  public void testRemoteEditNotifiesOncePerBatch() {
    configureTuning(true);
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> gains =
        Gains.angularPosition(Volts)
            .withPrefix("Tuned")
            .withKP(Volts.per(Rotations).ofNative(1.0))
            .withKD(Volts.per(RotationsPerSecond).ofNative(0.1))
            .build();
    AtomicInteger counter = new AtomicInteger(0);
    gains.onChange(g -> counter.incrementAndGet());

    // No edits, no notification
    TunableRegistry.update();
    assertEquals(0, counter.get());

    // Two gains edited in the same loop produce one notification with both new values
    backend.setDouble("/Tuned/kP", 2.0);
    backend.setDouble("/Tuned/kD", 0.2);
    TunableRegistry.update();
    assertEquals(1, counter.get());
    assertEquals(2.0, gains.getKP(Volts.per(Rotations)), EPSILON);
    assertEquals(0.2, gains.getKD(Volts.per(RotationsPerSecond)), EPSILON);

    // Setting the same value again is not a change
    backend.setDouble("/Tuned/kP", 2.0);
    TunableRegistry.update();
    assertEquals(1, counter.get());
  }

  @Test
  public void testImmutableWhenNotTuning() {
    configureTuning(false);
    Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> gains =
        Gains.angularPosition(Volts)
            .withPrefix("Locked")
            .withKP(Volts.per(Rotations).ofNative(1.0))
            .build();
    AtomicInteger counter = new AtomicInteger(0);
    gains.onChange(g -> counter.incrementAndGet());

    backend.setDouble("/Locked/kP", 5.0);
    TunableRegistry.update();
    assertEquals(1.0, gains.getKP(Volts.per(Rotations)), EPSILON);
    assertEquals(0, counter.get());
  }

  private void assertUnit(String path, Unit unit) {
    assertEquals(
        unit.symbol(), backend.getValue(path + GompeiTunables.UNIT_SUFFIX, String.class), path);
    String properties = backend.getTunable(path).getConfig().getProperties();
    assertTrue(
        properties.contains("\"" + GompeiTunables.UNIT_PROPERTY + "\":\"" + unit.symbol() + "\""),
        path + " properties " + properties);
  }
}
