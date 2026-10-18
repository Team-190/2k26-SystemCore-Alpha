package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

public class MechanismConversionTest extends GompeiLibTestBase {
  @Test
  public void testAngularIsTheIdentity() {
    var conversion = MechanismConversion.angular();

    assertEquals(0.25, conversion.toMotorPosition(Rotations.of(0.25)).in(Rotations), 1e-9);
    assertEquals(
        2.0, conversion.toMotorVelocity(RotationsPerSecond.of(2.0)).in(RotationsPerSecond), 1e-9);
    assertEquals(
        3.0,
        conversion
            .toMotorAcceleration(RotationsPerSecondPerSecond.of(3.0))
            .in(RotationsPerSecondPerSecond),
        1e-9);

    Angle position = (Angle) conversion.fromMotorPosition(Degrees.of(30));
    assertEquals(30.0, position.in(Degrees), 1e-9);
    assertEquals(
        1.5, conversion.fromMotorVelocity(RadiansPerSecond.of(1.5)).in(RadiansPerSecond), 1e-9);
    assertEquals(
        4.0,
        conversion
            .fromMotorAcceleration(RadiansPerSecondPerSecond.of(4.0))
            .in(RadiansPerSecondPerSecond),
        1e-9);
  }

  @Test
  public void testDrumScalesByItsRadius() {
    var conversion = MechanismConversion.drum(Meters.of(0.05));

    assertEquals(20.0, conversion.toMotorPosition(Meters.of(1.0)).in(Radians), 1e-9);
    assertEquals(
        2.0, conversion.toMotorVelocity(MetersPerSecond.of(0.1)).in(RadiansPerSecond), 1e-9);
    assertEquals(
        4.0,
        conversion
            .toMotorAcceleration(MetersPerSecondPerSecond.of(0.2))
            .in(RadiansPerSecondPerSecond),
        1e-9);

    Distance travel = (Distance) conversion.fromMotorPosition(Radians.of(10));
    assertEquals(0.5, travel.in(Meters), 1e-9);
    LinearVelocity speed = (LinearVelocity) conversion.fromMotorVelocity(RadiansPerSecond.of(2));
    assertEquals(0.1, speed.in(MetersPerSecond), 1e-9);
    assertEquals(
        0.15,
        conversion
            .fromMotorAcceleration(RadiansPerSecondPerSecond.of(3))
            .in(MetersPerSecondPerSecond),
        1e-9);
  }

  @Test
  public void testRoundTripsThroughTheMotorSide() {
    var conversion = MechanismConversion.drum(Inches.of(1.0));
    Distance travel = Inches.of(7.5);
    assertEquals(
        7.5, conversion.fromMotorPosition(conversion.toMotorPosition(travel)).in(Inches), 1e-9);
  }

  @Test
  public void testBuildsTrapezoidalProfiles() {
    var conversion = MechanismConversion.drum(Meters.of(0.5));
    MotionProfileConfig profile =
        conversion.trapezoidal(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(2.0));
    assertEquals(
        MotionProfileConfig.trapezoidal(
            RadiansPerSecond.of(2.0), RadiansPerSecondPerSecond.of(4.0)),
        profile);
  }

  @Test
  public void shouldRejectANonPositiveDrumRadius() {
    assertThrows(IllegalArgumentException.class, () -> MechanismConversion.drum(Meters.of(0)));
    assertThrows(IllegalArgumentException.class, () -> MechanismConversion.drum(Meters.of(-1)));
    assertThrows(
        IllegalArgumentException.class, () -> MechanismConversion.drum(Meters.of(Double.NaN)));
  }
}
