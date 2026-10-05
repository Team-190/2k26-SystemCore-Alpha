package first.robot.v3_Horse_CV2_TR.commands;

import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoAlignConstants;
import first.robot.FieldConstants;
import first.robot.util.AllianceFlipUtil;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.util.Pair;

public final class V3_Horse_CV2_TRDriveCommands {

  @Setter private static double lastCardinalDirection = 0.0;
  @Getter private static double slowFactor = 0.5;

  /**
   * A command that drives a SwerveDrive using joystick input.
   *
   * @param drive The SwerveDrive to control.
   * @param driveConstants The constants for the SwerveDrive.
   * @param xSupplier The supplier of the x-axis joystick input.
   * @param ySupplier The supplier of the y-axis joystick input.
   * @param omegaSupplier The supplier of the omega joystick input.
   * @param rotationSupplier The supplier of the rotation of the robot.
   * @param hijackXSuppliers A list of pairs of boolean suppliers and double suppliers for hijacking
   *     the x velocity. If the boolean supplier is true, the x velocity will be set to the value of
   *     the double supplier. If multiple boolean suppliers are true, the first one in the list will
   *     take precedence.
   * @param hijackYSuppliers A list of pairs of boolean suppliers and double suppliers for hijacking
   *     the y velocity. If the boolean supplier is true, the y velocity will be set to the value of
   *     the double supplier. If multiple boolean suppliers are true, the first one in the list will
   *     take precedence.
   * @param hijackOmegaSuppliers A list of pairs of boolean suppliers and double suppliers for
   *     hijacking the omega velocity. If the boolean supplier is true, the omega velocity will be
   *     set to the value of the double supplier. If multiple boolean suppliers are true, the first
   *     one in the list will take precedence.
   * @return A command that drives the SwerveDrive.
   */
  @Trace
  public static Command joystickDrive(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier,
      List<Pair<BooleanSupplier, DoubleSupplier>> hijackXSuppliers,
      List<Pair<BooleanSupplier, DoubleSupplier>> hijackYSuppliers,
      List<Pair<BooleanSupplier, DoubleSupplier>> hijackOmegaSuppliers,
      BooleanSupplier slowMode,
      DoubleSupplier slowFactor) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()),
                  driveConstants.driverDeadband);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double omega =
              MathUtil.applyDeadband(omegaSupplier.getAsDouble(), driveConstants.driverDeadband);
          linearMagnitude *= linearMagnitude;

          // Calculate new linear velocities

          double fieldRelativeXVel =
              linearMagnitude * linearDirection.getCos() * drive.getMaxLinearSpeedMetersPerSec();
          double fieldRelativeYVel =
              linearMagnitude * linearDirection.getSin() * drive.getMaxLinearSpeedMetersPerSec();

          double angular = omega * drive.getMaxAngularSpeedRadPerSec();

          fieldRelativeXVel =
              hijackXSuppliers.stream()
                  .filter(pair -> pair.getFirst().getAsBoolean())
                  .map(pair -> pair.getSecond().getAsDouble())
                  .findFirst()
                  .orElse(
                      slowMode.getAsBoolean()
                          ? (fieldRelativeXVel * slowFactor.getAsDouble())
                          : fieldRelativeXVel);

          fieldRelativeYVel =
              hijackYSuppliers.stream()
                  .filter(pair -> pair.getFirst().getAsBoolean())
                  .map(pair -> pair.getSecond().getAsDouble())
                  .findFirst()
                  .orElse(
                      slowMode.getAsBoolean()
                          ? (fieldRelativeYVel * slowFactor.getAsDouble())
                          : fieldRelativeYVel);

          angular =
              hijackOmegaSuppliers.stream()
                  .filter(pair -> pair.getFirst().getAsBoolean())
                  .map(pair -> pair.getSecond().getAsDouble())
                  .findFirst()
                  .orElse(slowMode.getAsBoolean() ? (angular * slowFactor.getAsDouble()) : angular);

          ChassisVelocities chassisSpeeds =
              new ChassisVelocities(fieldRelativeXVel, fieldRelativeYVel, angular)
                  .toRobotRelative(AllianceFlipUtil.apply(rotationSupplier.get()));

          Logger.recordOutput("Drive/JoystickDrive/chassisSpeeds", chassisSpeeds);

          drive.runVelocity(chassisSpeeds);
        },
        drive);
  }

  public static Command joystickDrive(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier) {
    return joystickDrive(
        drive,
        driveConstants,
        xSupplier,
        ySupplier,
        omegaSupplier,
        rotationSupplier,
        List.of(),
        List.of(),
        List.of(),
        () -> false,
        () -> 1);
  }

  public static Command joystickDriveRotationLock(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier,
      BooleanSupplier pointAtHub,
      DoubleSupplier hubSetpoint,
      DoubleSupplier hubFeedforward,
      BooleanSupplier cardinalDirectionAlign,
      Supplier<Rotation2d> cardinalDirection) {
    ProfiledPIDController omegaController = createTunedOmegaController(driveConstants);

    return joystickDrive(
        drive,
        driveConstants,
        xSupplier,
        ySupplier,
        omegaSupplier,
        rotationSupplier,
        List.of(),
        List.of(),
        List.of(
            Pair.of(
                pointAtHub,
                () ->
                    V3_Horse_CV2_TRAutoAlignCommands.calculate(
                            omegaController,
                            hubSetpoint.getAsDouble(),
                            rotationSupplier.get().getRadians(),
                            drive.getMeasuredChassisVelocities().omega)
                        + hubFeedforward.getAsDouble()),
            Pair.of(
                cardinalDirectionAlign,
                () ->
                    V3_Horse_CV2_TRAutoAlignCommands.calculate(
                        omegaController,
                        cardinalDirection.get().getRadians(),
                        rotationSupplier.get().getRadians(),
                        drive.getMeasuredChassisVelocities().omega))),
        () -> false,
        () -> 1.0);
  }

  public static Command joystickDriveRotationLock(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier,
      BooleanSupplier pointAtHub,
      DoubleSupplier hubSetpoint,
      DoubleSupplier hubFeedforward,
      BooleanSupplier cardinalDirectionAlign,
      Supplier<Rotation2d> cardinalDirection,
      BooleanSupplier climbSlowMode) {
    return joystickDriveRotationLock(
        drive,
        driveConstants,
        xSupplier,
        ySupplier,
        omegaSupplier,
        rotationSupplier,
        pointAtHub,
        hubSetpoint,
        hubFeedforward,
        cardinalDirectionAlign,
        cardinalDirection);
  }

  public static Command joystickDriveWithCardinalDirection(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<Rotation2d> rotationSupplier,
      BooleanSupplier cardinalDirectionAlign,
      BooleanSupplier slowMode) {
    ProfiledPIDController omegaController = createTunedOmegaController(driveConstants);

    return joystickDrive(
        drive,
        driveConstants,
        xSupplier,
        ySupplier,
        omegaSupplier,
        rotationSupplier,
        List.of(),
        List.of(),
        List.of(
            Pair.of(
                cardinalDirectionAlign,
                () ->
                    V3_Horse_CV2_TRAutoAlignCommands.calculate(
                        omegaController,
                        lastCardinalDirection,
                        rotationSupplier.get().getRadians(),
                        drive.getMeasuredChassisVelocities().omega))),
        slowMode,
        () -> slowFactor);
  }

  public static Command incrementSlowFactor() {
    return Commands.runOnce(() -> slowFactor = Math.min(slowFactor + 0.05, 1.0));
  }

  public static Command decrementSlowFactor() {
    return Commands.runOnce(() -> slowFactor = Math.max(slowFactor - 0.05, 0.0));
  }

  public static Command rotateToAngle(
      SwerveDrive drive,
      SwerveDriveConstants driveConstants,
      Supplier<Rotation2d> currentRotation,
      Supplier<Rotation2d> targetRotation) {
    ProfiledPIDController omegaController = createOmegaController(driveConstants);

    return Commands.run(
        () ->
            drive.runVelocity(
                new ChassisVelocities(
                        0.0,
                        0.0,
                        V3_Horse_CV2_TRAutoAlignCommands.calculate(
                            omegaController,
                            targetRotation.get().getRadians(),
                            currentRotation.get().getRadians(),
                            drive.getMeasuredChassisVelocities().omega))
                    .toRobotRelative(AllianceFlipUtil.apply(currentRotation.get()))),
        drive);
  }

  public static boolean atAngle(Rotation2d currentRotation, Rotation2d targetRotation) {
    return Math.abs(currentRotation.minus(targetRotation).getRadians())
        <= Units.degreesToRadians(2.0);
  }

  public static Command inchMovement(SwerveDrive drive, double velocity, double time) {
    return Commands.run(() -> drive.runVelocity(new ChassisVelocities(0.0, velocity, 0.0)))
        .withTimeout(time);
  }

  public static Command stop(SwerveDrive drive) {
    return Commands.run(drive::stopWithX);
  }

  public static Command feedforwardCharacterization(SwerveDrive drive) {
    return new KSCharacterization(
        drive, drive::runCharacterization, drive::getFFCharacterizationVelocity);
  }

  public static Command autoAlignPoseCommand(
      SwerveDrive drive,
      Supplier<Pose2d> robotPoseSupplier,
      Pose2d targetPose,
      AutoAlignConstants constants) {
    return new V3_Horse_CV2_TRAutoAlignCommands(
        drive,
        targetPose,
        () -> true,
        robotPoseSupplier,
        constants,
        // Must be finite: alpha-7 TrapezoidProfile computes inf * 0 = NaN otherwise
        constants.xConstraints().maxAcceleration().get().in(MetersPerSecondPerSecond));
  }

  public static Command autoAlignTowerCommand(
      SwerveDrive drive, Supplier<Pose2d> robotPoseSupplier, AutoAlignConstants constants) {
    return autoAlignPoseCommand(
        drive,
        robotPoseSupplier,
        new Pose2d(FieldConstants.Tower.centerPoint, new Rotation2d()),
        constants);
  }

  public static Command aimAtHub(SwerveDrive drive, SwerveDriveConstants driveConstants) {
    ProfiledPIDController omegaController = createOmegaController(driveConstants);

    return Commands.run(
        () ->
            drive.runVelocity(
                new ChassisVelocities(
                        0.0,
                        0.0,
                        V3_Horse_CV2_TRAutoAlignCommands.calculate(
                            omegaController,
                            V3_Horse_CV2_TRRobotState.getAimAngle().getRadians(),
                            V3_Horse_CV2_TRRobotState.getHeading().getRadians(),
                            drive.getMeasuredChassisVelocities().omega))
                    .toRobotRelative(V3_Horse_CV2_TRRobotState.getHeading())));
  }

  public static Command wheelRadiusCharacterization(
      SwerveDrive drive, SwerveDriveConstants driveConstants) {
    double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // SwerveDrive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRawGyroRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRawGyroRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * driveConstants.driveConfig.driveBaseRadius())
                              / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static ProfiledPIDController createOmegaController(SwerveDriveConstants driveConstants) {
    AutoAlignConstants constants = driveConstants.autoAlignConstants;
    ProfiledPIDController omegaController =
        new ProfiledPIDController(
            constants.rotationGains().kP().get(),
            0.0,
            constants.rotationGains().kD().get(),
            new TrapezoidProfile.Constraints(
                constants.rotationConstraints().maxVelocity().get().in(RadiansPerSecond),
                // Must be finite: alpha-7 TrapezoidProfile computes inf * 0 = NaN otherwise
                constants
                    .rotationConstraints()
                    .maxAcceleration()
                    .get()
                    .in(RadiansPerSecondPerSecond)));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);
    omegaController.setTolerance(
        constants.rotationConstraints().goalTolerance().get().in(Radians), 0);
    return omegaController;
  }

  /** Creates an omega controller whose PID gains follow live edits to the rotation gains. */
  private static ProfiledPIDController createTunedOmegaController(
      SwerveDriveConstants driveConstants) {
    ProfiledPIDController omegaController = createOmegaController(driveConstants);
    driveConstants
        .autoAlignConstants
        .rotationGains()
        .onChange(g -> omegaController.setPID(g.getKP(), 0, g.getKD()));
    return omegaController;
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
