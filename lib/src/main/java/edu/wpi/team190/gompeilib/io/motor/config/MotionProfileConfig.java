package edu.wpi.team190.gompeilib.io.motor.config;

import static org.wpilib.units.Units.*;

import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Per;
import org.wpilib.units.measure.Velocity;

/**
 * Constraints for profiled requests that don't carry their own, at the gearbox output.
 *
 * @param cruiseVelocity the trapezoidal profile's cruise velocity, and the exponential profile's
 *     velocity limit (zero for none)
 * @param acceleration the trapezoidal profile's acceleration
 * @param jerk the trapezoidal profile's jerk (zero for none)
 * @param exponentialKV the exponential profile's velocity gain
 * @param exponentialKA the exponential profile's acceleration gain
 */
public record MotionProfileConfig(
    AngularVelocity cruiseVelocity,
    AngularAcceleration acceleration,
    Velocity<AngularAccelerationUnit> jerk,
    Per<VoltageUnit, AngularVelocityUnit> exponentialKV,
    Per<VoltageUnit, AngularAccelerationUnit> exponentialKA) {

  /** No profile constraints. */
  public static final MotionProfileConfig NONE =
      new MotionProfileConfig(
          RotationsPerSecond.of(0.0),
          RotationsPerSecondPerSecond.of(0.0),
          RotationsPerSecondPerSecond.per(Second).of(0.0),
          Volts.per(RotationsPerSecond).ofNative(0.0),
          Volts.per(RotationsPerSecondPerSecond).ofNative(0.0));

  /**
   * Returns a trapezoidal profile with no jerk limit and no exponential profile.
   *
   * @param cruiseVelocity the cruise velocity
   * @param acceleration the acceleration
   * @return the constraints
   */
  public static MotionProfileConfig trapezoidal(
      AngularVelocity cruiseVelocity, AngularAcceleration acceleration) {
    return new MotionProfileConfig(
        cruiseVelocity, acceleration, NONE.jerk(), NONE.exponentialKV(), NONE.exponentialKA());
  }
}
