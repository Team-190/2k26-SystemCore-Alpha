package edu.wpi.team190.gompeilib.io.physics;

import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;

/**
 * A pluggable physics model for simulated motors. Positions, velocities, and accelerations are at
 * the gearbox output.
 *
 * <p>Several simulated motors can share one mechanism. The first motor to {@link #claimDriver}
 * drives it: it applies voltage and steps the physics. The others read its state.
 */
public interface MechanismSim {
  /**
   * Sets the voltage applied to every motor in the gearbox.
   *
   * @param voltage the voltage
   */
  void setInputVoltage(Voltage voltage);

  /**
   * Returns the voltage last applied.
   *
   * @return the voltage
   */
  Voltage inputVoltage();

  /**
   * Steps the physics forward.
   *
   * @param dt the time step
   */
  void update(Time dt);

  /**
   * Returns the gearbox-output position.
   *
   * @return the position
   */
  Angle position();

  /**
   * Returns the gearbox-output velocity.
   *
   * @return the velocity
   */
  AngularVelocity velocity();

  /**
   * Returns the gearbox-output acceleration.
   *
   * @return the acceleration
   */
  AngularAcceleration acceleration();

  /**
   * Returns the current drawn by all of the gearbox's motors together.
   *
   * @return the total current
   */
  Current currentDraw();

  /**
   * Returns the motors driving the mechanism. Its motor count splits current between them.
   *
   * @return the gearbox
   */
  DCMotor gearbox();

  /**
   * Registers a motor with the mechanism. Registering the same motor again has no effect.
   *
   * @param motor the motor
   * @return true if this motor is the first to register, and so drives the mechanism
   */
  boolean claimDriver(Object motor);

  /**
   * Returns how many motors have registered. Each one is reported as drawing an equal share of
   * {@link #currentDraw()}.
   *
   * @return the number of registered motors, at least one
   */
  int motorCount();

  /**
   * Returns the motor that drives the mechanism.
   *
   * @return the first registered motor, or null if none has registered
   */
  Object driver();
}
