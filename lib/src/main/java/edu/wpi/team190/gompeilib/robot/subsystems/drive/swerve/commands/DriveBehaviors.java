package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static org.wpilib.units.Units.RadiansPerSecond;

import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;

/** The driving behaviors a joystick drive can bind to triggers. */
public final class DriveBehaviors {
  private DriveBehaviors() {}

  /** Overrides the forward velocity, in the request's frame. */
  public static DriveBehavior overrideX(Supplier<LinearVelocity> velocity) {
    return (context, request) -> request.overrideX(velocity.get());
  }

  /** Overrides the leftward velocity, in the request's frame. */
  public static DriveBehavior overrideY(Supplier<LinearVelocity> velocity) {
    return (context, request) -> request.overrideY(velocity.get());
  }

  /** Overrides the angular velocity. */
  public static DriveBehavior overrideOmega(Supplier<AngularVelocity> velocity) {
    return (context, request) -> request.overrideOmega(velocity.get());
  }

  /**
   * Turns the robot to a field heading with the drive's rotation gains and constraints.
   *
   * @param constants the drive's constants
   * @param heading the target field heading
   */
  public static DriveBehavior faceHeading(
      SwerveDriveConstants constants, Supplier<Rotation2d> heading) {
    return faceHeading(constants, heading, () -> RadiansPerSecond.of(0.0));
  }

  /**
   * Turns the robot to a field heading with the drive's rotation gains and constraints, adding a
   * feedforward to the controller's output.
   *
   * @param constants the drive's constants
   * @param heading the target field heading
   * @param feedforward the angular velocity added to the controller's output
   */
  public static DriveBehavior faceHeading(
      SwerveDriveConstants constants,
      Supplier<Rotation2d> heading,
      Supplier<AngularVelocity> feedforward) {
    return new FaceHeading(constants.autoAlignConstants, heading, feedforward);
  }

  /**
   * Scales the linear and angular stick input by one factor, as in a slow mode.
   *
   * @param factor the factor, read every loop so it can be tuned at runtime
   */
  public static DriveBehavior scale(DoubleSupplier factor) {
    return scale(factor, factor);
  }

  /**
   * Scales the linear and angular stick input separately.
   *
   * @param linearFactor the factor for the forward and leftward input
   * @param angularFactor the factor for the angular input
   */
  public static DriveBehavior scale(DoubleSupplier linearFactor, DoubleSupplier angularFactor) {
    return (context, request) ->
        request.scaleInput(linearFactor.getAsDouble(), angularFactor.getAsDouble());
  }

  /** Drives in the robot's frame instead of the driver's field-relative frame. */
  public static DriveBehavior robotRelative() {
    return (context, request) -> request.setRobotRelative();
  }
}
