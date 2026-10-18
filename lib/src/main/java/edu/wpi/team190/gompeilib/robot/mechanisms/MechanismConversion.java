package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import java.util.Objects;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

/**
 * A linear map between a mechanism's units and the motor side's gearbox-output rotation. Position,
 * velocity, and acceleration share one scale, so the same conversion turns goals into requests,
 * telemetry into readings, constraints into motion profiles, and gains into gains at the gearbox
 * output.
 *
 * @param <P> the mechanism's position unit
 * @param <V> the mechanism's velocity unit
 * @param <A> the mechanism's acceleration unit
 */
public final class MechanismConversion<P extends Unit, V extends Unit, A extends Unit> {
  private final P positionUnit;
  private final V velocityUnit;
  private final A accelerationUnit;
  private final double baseUnitsPerRadian;

  private MechanismConversion(
      P positionUnit, V velocityUnit, A accelerationUnit, double baseUnitsPerRadian) {
    this.positionUnit = Objects.requireNonNull(positionUnit, "positionUnit");
    this.velocityUnit = Objects.requireNonNull(velocityUnit, "velocityUnit");
    this.accelerationUnit = Objects.requireNonNull(accelerationUnit, "accelerationUnit");
    this.baseUnitsPerRadian = baseUnitsPerRadian;
  }

  /**
   * Returns the identity conversion, for a mechanism measured as the gearbox output's rotation (an
   * arm, a hood, a flywheel, a roller).
   *
   * @return the conversion
   */
  public static MechanismConversion<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit>
      angular() {
    return new MechanismConversion<>(Radians, RadiansPerSecond, RadiansPerSecondPerSecond, 1.0);
  }

  /**
   * Returns the conversion for travel driven through a drum, where one radian of the gearbox output
   * moves the mechanism one drum radius.
   *
   * @param radius the drum's radius
   * @return the conversion
   * @throws IllegalArgumentException if the radius isn't positive
   */
  public static MechanismConversion<DistanceUnit, LinearVelocityUnit, LinearAccelerationUnit> drum(
      Distance radius) {
    double meters = radius.in(Meters);
    if (!(meters > 0.0)) {
      throw new IllegalArgumentException("Drum radius must be positive, was " + radius);
    }
    return new MechanismConversion<>(Meters, MetersPerSecond, MetersPerSecondPerSecond, meters);
  }

  public Angle toMotorPosition(Measure<P> position) {
    return Radians.of(position.baseUnitMagnitude() / baseUnitsPerRadian);
  }

  public AngularVelocity toMotorVelocity(Measure<V> velocity) {
    return RadiansPerSecond.of(velocity.baseUnitMagnitude() / baseUnitsPerRadian);
  }

  public AngularAcceleration toMotorAcceleration(Measure<A> acceleration) {
    return RadiansPerSecondPerSecond.of(acceleration.baseUnitMagnitude() / baseUnitsPerRadian);
  }

  // ofBaseUnits returns the unit's own measure type, which is a Measure of that unit
  @SuppressWarnings("unchecked")
  public Measure<P> fromMotorPosition(Angle position) {
    return (Measure<P>) positionUnit.ofBaseUnits(position.in(Radians) * baseUnitsPerRadian);
  }

  @SuppressWarnings("unchecked")
  public Measure<V> fromMotorVelocity(AngularVelocity velocity) {
    return (Measure<V>)
        velocityUnit.ofBaseUnits(velocity.in(RadiansPerSecond) * baseUnitsPerRadian);
  }

  @SuppressWarnings("unchecked")
  public Measure<A> fromMotorAcceleration(AngularAcceleration acceleration) {
    return (Measure<A>)
        accelerationUnit.ofBaseUnits(
            acceleration.in(RadiansPerSecondPerSecond) * baseUnitsPerRadian);
  }

  /**
   * Returns position gains declared in the mechanism's units as gains per gearbox-output radian.
   * The result is a live view: it follows dashboard edits to the declared gains.
   *
   * @param gains the gains, per mechanism position
   * @param <O> the output unit
   * @return the gains at the gearbox output
   */
  public <O extends Unit> Gains<O, AngleUnit, AngularVelocityUnit> toMotorPositionGains(
      Gains<O, P, V> gains) {
    return gains.view(Radians, RadiansPerSecond, baseUnitsPerRadian);
  }

  /**
   * Returns velocity gains declared in the mechanism's units as gains per gearbox-output radian per
   * second. The result is a live view: it follows dashboard edits to the declared gains.
   *
   * @param gains the gains, per mechanism velocity
   * @param <O> the output unit
   * @return the gains at the gearbox output
   */
  public <O extends Unit> Gains<O, AngularVelocityUnit, AngularVelocityUnit> toMotorVelocityGains(
      Gains<O, V, V> gains) {
    return gains.view(RadiansPerSecond, RadiansPerSecond, baseUnitsPerRadian);
  }

  /**
   * Returns a trapezoidal motion profile with the given mechanism-side limits.
   *
   * @param maxVelocity the cruise velocity
   * @param maxAcceleration the acceleration
   * @return the profile, at the gearbox output
   */
  public MotionProfileConfig trapezoidal(Measure<V> maxVelocity, Measure<A> maxAcceleration) {
    return MotionProfileConfig.trapezoidal(
        toMotorVelocity(maxVelocity), toMotorAcceleration(maxAcceleration));
  }
}
