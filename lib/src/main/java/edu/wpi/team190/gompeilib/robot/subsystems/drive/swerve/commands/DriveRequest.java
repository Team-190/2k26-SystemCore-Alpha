package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import java.util.EnumSet;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

/**
 * The chassis velocities a joystick drive commands this loop, as its active behaviors adjust them.
 *
 * <p>It starts from the shaped stick input. Behaviors then run in registration order and may:
 *
 * <ul>
 *   <li>override an axis with their own velocity; the first override on an axis wins and later ones
 *       on the same axis are ignored,
 *   <li>scale the axes that still carry stick input; an override's output is never scaled,
 *       whichever order the two were registered in,
 *   <li>switch the request from the driver's field-relative frame to the robot's frame.
 * </ul>
 */
public final class DriveRequest {
  /** One commanded axis of the chassis. */
  public enum Axis {
    X,
    Y,
    OMEGA
  }

  private LinearVelocity vx;
  private LinearVelocity vy;
  private AngularVelocity omega;
  private final EnumSet<Axis> overridden = EnumSet.noneOf(Axis.class);
  private boolean fieldRelative = true;

  /**
   * Creates a request from stick input, field-relative and with no axis overridden.
   *
   * @param vx forward velocity
   * @param vy leftward velocity
   * @param omega counterclockwise angular velocity
   */
  public DriveRequest(LinearVelocity vx, LinearVelocity vy, AngularVelocity omega) {
    this.vx = vx;
    this.vy = vy;
    this.omega = omega;
  }

  public LinearVelocity vx() {
    return vx;
  }

  public LinearVelocity vy() {
    return vy;
  }

  public AngularVelocity omega() {
    return omega;
  }

  /** Whether a behavior has already overridden the axis this loop. */
  public boolean isOverridden(Axis axis) {
    return overridden.contains(axis);
  }

  /** Whether the request is in the driver's field-relative frame rather than the robot's. */
  public boolean isFieldRelative() {
    return fieldRelative;
  }

  /**
   * Overrides the forward velocity, unless an earlier behavior already did.
   *
   * @return whether this override took effect
   */
  public boolean overrideX(LinearVelocity velocity) {
    if (!overridden.add(Axis.X)) return false;
    vx = velocity;
    return true;
  }

  /**
   * Overrides the leftward velocity, unless an earlier behavior already did.
   *
   * @return whether this override took effect
   */
  public boolean overrideY(LinearVelocity velocity) {
    if (!overridden.add(Axis.Y)) return false;
    vy = velocity;
    return true;
  }

  /**
   * Overrides the angular velocity, unless an earlier behavior already did.
   *
   * @return whether this override took effect
   */
  public boolean overrideOmega(AngularVelocity velocity) {
    if (!overridden.add(Axis.OMEGA)) return false;
    omega = velocity;
    return true;
  }

  /**
   * Scales the axes that still carry stick input. Overridden axes are left alone.
   *
   * @param linearFactor factor for the forward and leftward velocities
   * @param angularFactor factor for the angular velocity
   */
  public void scaleInput(double linearFactor, double angularFactor) {
    if (!isOverridden(Axis.X)) vx = vx.times(linearFactor);
    if (!isOverridden(Axis.Y)) vy = vy.times(linearFactor);
    if (!isOverridden(Axis.OMEGA)) omega = omega.times(angularFactor);
  }

  /** Commands the request in the robot's frame instead of the driver's field-relative frame. */
  public void setRobotRelative() {
    fieldRelative = false;
  }

  /**
   * Converts the request into robot-relative chassis velocities.
   *
   * @param driverHeading the robot's heading in the driver's frame, used when field-relative
   * @return the robot-relative chassis velocities
   */
  public ChassisVelocities toRobotRelative(Rotation2d driverHeading) {
    ChassisVelocities velocities =
        new ChassisVelocities(
            vx.in(MetersPerSecond), vy.in(MetersPerSecond), omega.in(RadiansPerSecond));
    return fieldRelative ? velocities.toRobotRelative(driverHeading) : velocities;
  }
}
