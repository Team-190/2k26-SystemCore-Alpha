package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;

import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.util.MathUtil;

/**
 * Drives a swerve drive from the driver's sticks, with driving behaviors bound to triggers.
 *
 * <p>Every loop it deadbands the sticks, squares the linear magnitude, scales the input to the
 * drive's max speeds, lets every held behavior adjust the request in registration order (see {@link
 * DriveRequest} for precedence), converts from the driver's frame to the robot's, and logs the
 * commanded velocities. Adding a behavior is a new {@link Builder#with} binding; it never changes
 * this command's signature.
 */
public final class JoystickDrive extends Command {
  private static final class Binding {
    private final BooleanSupplier trigger;
    private final DriveBehavior behavior;
    private boolean held = false;

    private Binding(BooleanSupplier trigger, DriveBehavior behavior) {
      this.trigger = trigger;
      this.behavior = behavior;
    }
  }

  private final SwerveDrive drive;
  private final DriveSticks sticks;
  private final double deadband;
  private final Supplier<Rotation2d> heading;
  private final Supplier<Rotation2d> driverPerspective;
  private final List<Binding> bindings;

  private JoystickDrive(Builder builder) {
    drive = builder.drive;
    sticks = builder.sticks;
    deadband = builder.deadband;
    heading = builder.heading;
    driverPerspective = builder.driverPerspective;
    bindings = List.copyOf(builder.bindings);
    addRequirements(drive);
  }

  /**
   * Shapes stick input into a field-relative request: deadband on the linear magnitude and on
   * omega, the linear magnitude squared, and both scaled to the max speeds.
   */
  static DriveRequest shape(
      DriveSticks sticks,
      double deadband,
      double maxLinearMetersPerSec,
      double maxAngularRadPerSec) {
    double x = sticks.x().getAsDouble();
    double y = sticks.y().getAsDouble();
    double rawMagnitude = Math.hypot(x, y);
    double magnitude = MathUtil.applyDeadband(rawMagnitude, deadband);
    magnitude *= magnitude;
    double scale = rawMagnitude > 0.0 ? magnitude * maxLinearMetersPerSec / rawMagnitude : 0.0;
    double omega = MathUtil.applyDeadband(sticks.omega().getAsDouble(), deadband);
    return new DriveRequest(
        MetersPerSecond.of(x * scale),
        MetersPerSecond.of(y * scale),
        RadiansPerSecond.of(omega * maxAngularRadPerSec));
  }

  @Override
  public void initialize() {
    for (Binding binding : bindings) binding.held = false;
  }

  @Override
  public void execute() {
    DriveBehavior.Context context =
        new DriveBehavior.Context(heading.get(), drive.getMeasuredChassisVelocities());
    DriveRequest request =
        shape(
            sticks,
            deadband,
            drive.getMaxLinearSpeedMetersPerSec(),
            drive.getMaxAngularSpeedRadPerSec());
    for (Binding binding : bindings) {
      boolean held = binding.trigger.getAsBoolean();
      if (held && !binding.held) binding.behavior.engage(context);
      binding.held = held;
      if (held) binding.behavior.apply(context, request);
    }

    ChassisVelocities velocities =
        request.toRobotRelative(context.heading().plus(driverPerspective.get()));
    Logger.recordOutput("Drive/JoystickDrive/chassisSpeeds", velocities);
    drive.runVelocity(velocities);
  }

  /** Builds a {@link JoystickDrive}. Start one with {@link SwerveDriveCommands#joystickDrive}. */
  public static final class Builder {
    private final SwerveDrive drive;
    private final DriveSticks sticks;
    private final double deadband;
    private final Supplier<Rotation2d> heading;
    private Supplier<Rotation2d> driverPerspective = () -> Rotation2d.ZERO;
    private final List<Binding> bindings = new ArrayList<>();

    Builder(SwerveDrive drive, DriveSticks sticks, double deadband, Supplier<Rotation2d> heading) {
      this.drive = drive;
      this.sticks = sticks;
      this.deadband = deadband;
      this.heading = heading;
    }

    /**
     * Sets which way the driver faces in field coordinates, read every loop. Field-relative input
     * is driven in this frame: pass the field's 180 degrees when the driver stands on the far side,
     * such as on the red alliance.
     */
    public Builder withDriverPerspective(Supplier<Rotation2d> perspective) {
      driverPerspective = perspective;
      return this;
    }

    /**
     * Runs a behavior while its trigger is held. Behaviors run in the order they're bound.
     *
     * @param trigger whether the behavior is held, such as a controller button
     * @param behavior the behavior to run
     */
    public Builder with(BooleanSupplier trigger, DriveBehavior behavior) {
      bindings.add(new Binding(trigger, behavior));
      return this;
    }

    public JoystickDrive build() {
      return new JoystickDrive(this);
    }
  }
}
