package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants.AutoAlignConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands.DriveRequest.Axis;
import java.util.function.Supplier;
import lombok.AccessLevel;
import lombok.Getter;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Overrides the angular velocity with a profiled PID toward a target field heading, plus an
 * optional feedforward.
 *
 * <p>Whenever it takes over the angular axis (its trigger engages, or an earlier override on the
 * axis lets go), it restarts its profile from the current heading and angular velocity. Without
 * that, the profile resumes from the setpoint left over from the last time it ran, and the robot
 * swings back toward the old heading first. Its PID gains follow live edits to the drive's rotation
 * gains.
 */
public final class FaceHeading implements DriveBehavior {
  @Getter(AccessLevel.PACKAGE)
  private final ProfiledPIDController controller;

  private final Supplier<Rotation2d> target;
  private final Supplier<AngularVelocity> feedforward;
  private boolean inControl = false;

  FaceHeading(
      AutoAlignConstants constants,
      Supplier<Rotation2d> target,
      Supplier<AngularVelocity> feedforward) {
    this.target = target;
    this.feedforward = feedforward;
    PIDConstants pid = SwerveDrive.rotationConstants(constants.rotationGains());
    controller =
        new ProfiledPIDController(
            pid.kP,
            0.0,
            pid.kD,
            new TrapezoidProfile.Constraints(
                constants.rotationConstraints().maxVelocity().get().in(RadiansPerSecond),
                // Must be finite: alpha-7 TrapezoidProfile computes inf * 0 = NaN otherwise
                constants
                    .rotationConstraints()
                    .maxAcceleration()
                    .get()
                    .in(RadiansPerSecondPerSecond)));
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(constants.rotationConstraints().goalTolerance().get().in(Radians), 0);
    constants
        .rotationGains()
        .onChange(
            gains -> {
              PIDConstants updated = SwerveDrive.rotationConstants(gains);
              controller.setPID(updated.kP, 0.0, updated.kD);
            });
  }

  @Override
  public void engage(Context context) {
    inControl = false;
  }

  @Override
  public void apply(Context context, DriveRequest request) {
    if (request.isOverridden(Axis.OMEGA)) {
      inControl = false;
      return;
    }
    double measurement = context.heading().getRadians();
    double velocity = context.measuredVelocities().omega;
    if (!inControl) controller.reset(measurement, velocity);
    inControl = true;

    double output = controller.calculate(measurement, target.get().getRadians());
    request.overrideOmega(RadiansPerSecond.of(output).plus(feedforward.get()));
  }
}
