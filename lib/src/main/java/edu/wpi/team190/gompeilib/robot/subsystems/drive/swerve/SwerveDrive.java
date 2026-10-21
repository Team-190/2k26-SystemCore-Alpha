// Copyright 2021-2024 FRC 6328
package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve;

import static org.wpilib.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Sample;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.gyro.interfaces.GyroIO;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import edu.wpi.team190.gompeilib.logging.Trace;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.IntStream;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.util.Units;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.MultUnit;

public class SwerveDrive extends SubsystemBase {
  private final SwerveDriveConstants driveConstants;

  private final GyroIO gyroIO;
  private final GyroTelemetryAutoLogged gyroInputs;
  private final SwerveModule[] modules;

  private final LinearFilter xFilter;
  private final LinearFilter yFilter;
  private double filteredX;
  private double filteredY;

  private final SwerveDriveKinematics kinematics;
  @Getter private Rotation2d rawGyroRotation;
  private final SwerveModulePosition[] lastModulePositions;
  @Getter private ChassisVelocities measuredChassisVelocities;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final PIDController autoXController;
  private final PIDController autoYController;
  private final PIDController autoHeadingController;

  private RobotConfig config;

  public SwerveDrive(
      SwerveDriveConstants driveConstants,
      GyroIO gyroIO,
      SwerveModuleHardware flModule,
      SwerveModuleHardware frModule,
      SwerveModuleHardware blModule,
      SwerveModuleHardware brModule,
      Supplier<Pose2d> robotPoseSupplier,
      Consumer<Pose2d> resetPoseConsumer) {
    this.driveConstants = driveConstants;
    // The modules pick volts or torque current at runtime, so check the gains match before use.
    SlotConfig.requireOutputType(
        SlotConfig.velocity(driveConstants.driveGains),
        driveConstants.driveConfig.driveClosedLoopOutputType(),
        "Drive");
    SlotConfig.requireOutputType(
        SlotConfig.position(driveConstants.turnGains),
        driveConstants.driveConfig.steerClosedLoopOutputType(),
        "Turn");
    this.gyroIO = gyroIO;
    gyroInputs = new GyroTelemetryAutoLogged();
    modules = new SwerveModule[4]; // FL, FR, BL, BR
    modules[0] =
        new SwerveModule(driveConstants, driveConstants.driveConfig.frontLeft(), flModule, 0);
    modules[1] =
        new SwerveModule(driveConstants, driveConstants.driveConfig.frontRight(), frModule, 1);
    modules[2] =
        new SwerveModule(driveConstants, driveConstants.driveConfig.backLeft(), blModule, 2);
    modules[3] =
        new SwerveModule(driveConstants, driveConstants.driveConfig.backRight(), brModule, 3);

    xFilter = LinearFilter.movingAverage(10);
    yFilter = LinearFilter.movingAverage(10);
    filteredX = 0;
    filteredY = 0;

    kinematics = this.driveConstants.driveConfig.kinematics();
    rawGyroRotation = new Rotation2d();
    lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    this.robotPoseSupplier = robotPoseSupplier;

    // Start the odometry thread (no-op if no device registered a high-frequency signal)
    PhoenixOdometryThread.getInstance(driveConstants).start();

    autoHeadingController = new PIDController(0.0, 0.0, 0.0, GompeiLib.getLoopPeriod());
    autoXController = new PIDController(0.0, 0.0, 0.0, GompeiLib.getLoopPeriod());
    autoYController = new PIDController(0.0, 0.0, 0.0, GompeiLib.getLoopPeriod());
    setAutoControllers(driveConstants.autoTranslationGains, driveConstants.autoRotationGains);

    autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    autoHeadingController.setTolerance(Units.degreesToRadians(1.0));

    driveConstants.driveGains.onChange(gains -> updateDriveGains(gains, GainSlot.ZERO));
    driveConstants.turnGains.onChange(gains -> updateTurnGains(gains, GainSlot.ZERO));
    driveConstants.autoTranslationGains.onChange(
        gains -> setAutoControllers(gains, driveConstants.autoRotationGains));
    driveConstants.autoRotationGains.onChange(
        gains -> setAutoControllers(driveConstants.autoTranslationGains, gains));

    measuredChassisVelocities = new ChassisVelocities();

    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this.robotPoseSupplier,
          resetPoseConsumer, // resetPose
          () -> getChassisVelocities(), // get robotRelativeSpeeds
          (speeds, feedforwards) -> {
            // Retained from Choreo; switch to runVelocityTorque(speeds, forces) if needed.
            @SuppressWarnings("unused")
            List<Vector<N2>> forces =
                IntStream.range(0, 4)
                    .mapToObj(
                        i ->
                            VecBuilder.fill(
                                feedforwards.robotRelativeForcesXNewtons()[i],
                                feedforwards.robotRelativeForcesYNewtons()[i]))
                    .toList();

            runVelocity(speeds);
          },
          new PPHolonomicDriveController(
              translationConstants(driveConstants.autoTranslationGains),
              rotationConstants(driveConstants.autoRotationGains)),
          config,
          () -> {
            var alliance = DriverStationBackend.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == Alliance.RED;
            }
            return false;
          },
          this);
    } catch (Exception e) {
      throw new RuntimeException("Failed to load PathPlanner robot config", e);
    }
  }

  @Trace
  public void periodic() {
    driveConstants.reentrantLock.lock();

    gyroIO.updateInputs(gyroInputs);

    for (int i = 0; i < 4; i++) {
      modules[i].updateInputs();
    }

    driveConstants.reentrantLock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }

    // Stop moving when disabled
    if (RobotState.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    Logger.recordOutput("SwerveStates/Measured", getModuleStates());
    Logger.recordOutput("SwerveChassisVelocities/Measured", measuredChassisVelocities);

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation =
            new Rotation2d(
                i < gyroInputs.odometryYawPositions.length
                    ? gyroInputs.odometryYawPositions[i]
                    : gyroInputs.yawPosition);
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      ChassisVelocities chassisSpeeds = kinematics.toChassisVelocities(getModuleStates());
      measuredChassisVelocities = chassisSpeeds;
      Translation2d rawFieldRelativeVelocity =
          new Translation2d(chassisSpeeds.vx, chassisSpeeds.vy).rotateBy(getRawGyroRotation());

      filteredX = xFilter.calculate(rawFieldRelativeVelocity.getX());
      filteredY = yFilter.calculate(rawFieldRelativeVelocity.getY());
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  @Trace
  public void runVelocity(ChassisVelocities speeds) {
    // Calculate module setpoints
    ChassisVelocities optimizedSpeeds = speeds.discretize(GompeiLib.getLoopPeriod());
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(optimizedSpeeds);
    SwerveDriveKinematics.desaturateWheelVelocities(
        setpointStates, driveConstants.driveConfig.maxLinearVelocityMetersPerSecond());

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisVelocities/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      setpointStates[i] = modules[i].runSetpoint(setpointStates[i], new SwerveModuleVelocity());
    }

    // Log optimized setpoints
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive at the desired velocity and torque.
   *
   * @param speeds Speeds in meters/sec
   */
  @Trace
  public void runVelocityTorque(ChassisVelocities speeds, List<Vector<N2>> forces) {
    if (forces.size() != 4) {
      throw new IllegalArgumentException("Forces array must have 4 elements");
    }
    // Calculate module setpoints
    ChassisVelocities optimizedSpeeds = speeds.discretize(GompeiLib.getLoopPeriod());
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(optimizedSpeeds);
    SwerveModuleVelocity[] setpointTorques = new SwerveModuleVelocity[4];
    SwerveDriveKinematics.desaturateWheelVelocities(
        setpointStates, driveConstants.driveConfig.maxLinearVelocityMetersPerSecond());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      Vector<N2> wheelDirection =
          VecBuilder.fill(setpointStates[i].angle.getCos(), setpointStates[i].angle.getSin());
      setpointTorques[i] =
          new SwerveModuleVelocity(
              forces.get(i).dot(wheelDirection)
                  * driveConstants.driveConfig.frontLeft().driveMotor().gearRatio,
              setpointStates[i].angle);

      setpointStates[i] = setpointStates[i].optimize(modules[i].getAngle());
      setpointTorques[i] = setpointTorques[i].optimize(modules[i].getAngle());

      setpointStates[i] = modules[i].runSetpoint(setpointStates[i], setpointTorques[i]);
    }

    // Log optimized setpoints
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveStates/TorquesOptimized", setpointTorques);
  }

  /** Runs the drive in a straight line with an open-loop request to every drive motor. */
  @Trace
  public void runCharacterization(OpenLoopRequest request) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(request);
    }
  }

  /** Stops the drive. */
  @Trace
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  @Trace
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] =
          driveConstants.driveConfig.getModuleTranslations()[i].getAngle().orElse(Rotation2d.ZERO);
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @Trace
  private SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  @Trace
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @Trace
  private ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  @Trace
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /**
   * Returns the modules' average drive state at the wheel, for characterization.
   *
   * @param effort whether to report voltage or torque current as the effort
   * @return the average sample
   */
  @Trace
  public Sample getFFCharacterizationSample(OutputType effort) {
    double applied = 0.0;
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
    double backEmf = 0.0;
    for (SwerveModule module : modules) {
      Sample sample = module.getFFCharacterizationSample(effort);
      applied += sample.effort() / modules.length;
      position += sample.position() / modules.length;
      velocity += sample.velocity() / modules.length;
      acceleration += sample.acceleration() / modules.length;
      backEmf += sample.backEmf() / modules.length;
    }
    return new Sample(applied, position, velocity, acceleration, backEmf);
  }

  /** Returns the maximum linear speed in meters per sec. */
  @Trace
  public double getMaxLinearSpeedMetersPerSec() {
    return driveConstants.driveConfig.maxLinearVelocityMetersPerSecond();
  }

  /** Returns the maximum angular speed in radians per sec. */
  @Trace
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / driveConstants.driveConfig.driveBaseRadius();
  }

  /** Returns the field relative velocity in X and Y. */
  @Trace
  public Translation2d getFieldRelativeVelocity() {
    return new Translation2d(filteredX, filteredY);
  }

  /** Returns the current yaw velocity */
  @Trace
  public double getYawVelocity() {
    return gyroInputs.yawVelocity.in(RadiansPerSecond);
  }

  /**
   * Updates every module's drive motor gains for a slot.
   *
   * @param gains the gains to apply
   * @param slot the target gain slot
   */
  @Trace
  public void updateDriveGains(
      Gains<?, AngularVelocityUnit, AngularVelocityUnit> gains, GainSlot slot) {
    for (var module : modules) {
      module.updateDriveGains(gains, slot);
    }
  }

  /**
   * Updates every module's turn motor gains for a slot.
   *
   * @param gains the gains to apply
   * @param slot the target gain slot
   */
  @Trace
  public void updateTurnGains(Gains<?, AngleUnit, AngularVelocityUnit> gains, GainSlot slot) {
    for (var module : modules) {
      module.updateTurnGains(gains, slot);
    }
  }

  // TODO: restore choreoDrive(SwerveSample) once ChoreoLib ships a WPILib 2027 alpha build;
  // it has no compatible release yet, so the choreo dependency and this method are removed.

  /**
   * Sets the auto controllers' proportional and derivative gains.
   *
   * @param translationGains the translation gains
   * @param rotationGains the rotation gains
   */
  public void setAutoControllers(
      Gains<LinearVelocityUnit, DistanceUnit, LinearVelocityUnit> translationGains,
      Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit> rotationGains) {
    PIDConstants translation = translationConstants(translationGains);
    PIDConstants rotation = rotationConstants(rotationGains);
    autoXController.setPID(translation.kP, 0.0, translation.kD);
    autoYController.setPID(translation.kP, 0.0, translation.kD);
    autoHeadingController.setPID(rotation.kP, 0.0, rotation.kD);
  }

  /**
   * Reads translation gains in the units WPILib's and PathPlanner's controllers use: meters per
   * second of output per meter of error.
   *
   * @param gains the gains
   * @return the PID constants
   */
  public static PIDConstants translationConstants(
      Gains<LinearVelocityUnit, DistanceUnit, LinearVelocityUnit> gains) {
    return new PIDConstants(
        gains.getKP(MetersPerSecond.per(Meters)),
        gains.getKI(MetersPerSecond.per(MultUnit.combine(Meters, Seconds))),
        gains.getKD(MetersPerSecond.per(MetersPerSecond)));
  }

  /**
   * Reads rotation gains in the units WPILib's and PathPlanner's controllers use: radians per
   * second of output per radian of error.
   *
   * @param gains the gains
   * @return the PID constants
   */
  public static PIDConstants rotationConstants(
      Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit> gains) {
    return new PIDConstants(
        gains.getKP(RadiansPerSecond.per(Radians)),
        gains.getKI(RadiansPerSecond.per(MultUnit.combine(Radians, Seconds))),
        gains.getKD(RadiansPerSecond.per(RadiansPerSecond)));
  }

  /**
   * Updates current limits.
   *
   * @param driveCurrentLimit The drive current limit.
   * @param turnCurrentLimit The turn current limit.
   */
  @Trace
  public void updateCurrentLimits(double driveCurrentLimit, double turnCurrentLimit) {
    for (SwerveModule s : modules) {
      s.updateCurrentLimits(driveCurrentLimit, turnCurrentLimit);
    }
  }
}
