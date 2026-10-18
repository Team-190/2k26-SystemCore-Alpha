package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve;

import static org.wpilib.units.Units.Hertz;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import edu.wpi.team190.gompeilib.io.gyro.config.GyroConfig;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.system.DCMotor;
import org.wpilib.tunable.Tunable;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

@Builder(setterPrefix = "with")
public class SwerveDriveConstants {
  @Builder.Default public final ReentrantLock reentrantLock = new ReentrantLock();

  @NonNull public final DriveConfig driveConfig;

  /**
   * The drive motors' velocity gains, per wheel rotation; output must match the drive output type.
   */
  @NonNull public final Gains<?, AngularVelocityUnit, AngularVelocityUnit> driveGains;

  /**
   * The turn motors' position gains, per module rotation; output must match the steer output type.
   */
  @NonNull public final Gains<?, AngleUnit, AngularVelocityUnit> turnGains;

  /** PathPlanner's translation gains, from position error to chassis velocity. */
  @NonNull
  public final Gains<LinearVelocityUnit, DistanceUnit, LinearVelocityUnit> autoTranslationGains;

  /** PathPlanner's rotation gains, from heading error to chassis angular velocity. */
  @NonNull
  public final Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit> autoRotationGains;

  @NonNull public final AutoAlignConstants autoAlignConstants;

  @NonNull public final Double odometryFrequency;
  @NonNull public final Double driverDeadband;
  @NonNull public final Double operatorDeadband;

  /**
   * Returns the drive gyro's config: the Pigeon2 on the drive bus, with yaw published at the
   * odometry rate so the odometry thread can sample it.
   *
   * @return the gyro config
   */
  public GyroConfig gyroConfig() {
    return GyroConfig.builder()
        .withCanId(driveConfig.pigeon2Id())
        .withBus(driveConfig.canBus())
        .withSignalRates(
            new SignalRates(
                SignalRates.DEFAULT.control(),
                SignalRates.DEFAULT.diagnostic(),
                Map.of("yawPosition", Hertz.of(odometryFrequency))))
        .build();
  }

  @Builder(setterPrefix = "with")
  public record DriveConfig(
      @NonNull CANBusId canBus,
      @NonNull Integer pigeon2Id,
      @NonNull Double maxLinearVelocityMetersPerSecond,
      @NonNull Double wheelRadiusMeters,
      @NonNull DCMotor driveModel,
      @NonNull DCMotor turnModel,
      @NonNull SwerveModuleConfig frontLeft,
      @NonNull SwerveModuleConfig frontRight,
      @NonNull SwerveModuleConfig backLeft,
      @NonNull SwerveModuleConfig backRight,
      @NonNull OutputType driveClosedLoopOutputType,
      @NonNull OutputType steerClosedLoopOutputType,
      @NonNull Double bumperWidth,
      @NonNull Double bumperLength,
      @NonNull Double robotMassKilograms,
      @NonNull Double trackWidth,
      @NonNull Double robotMOI,
      @NonNull Double moduleCurrentLimit,
      @NonNull Double wheelCOF) {
    public Double driveBaseRadius() {
      return Math.hypot(
          (Math.abs(frontLeft.location().getX()) + Math.abs(frontRight.location().getX())) / 2.0,
          (Math.abs(frontLeft.location().getY()) + Math.abs(backLeft.location().getY())) / 2.0);
    }

    public Double maxAngularVelocity() {
      return maxLinearVelocityMetersPerSecond / driveBaseRadius();
    }

    public Translation2d[] getModuleTranslations() {
      return new Translation2d[] {
        frontLeft.location(), frontRight.location(), backLeft.location(), backRight.location()
      };
    }

    public SwerveDriveKinematics kinematics() {
      return new SwerveDriveKinematics(getModuleTranslations());
    }
  }

  @Builder(setterPrefix = "with")
  public record AutoAlignConstants(
      @NonNull Gains<LinearVelocityUnit, DistanceUnit, LinearVelocityUnit> xGains,
      @NonNull LinearConstraints xConstraints,
      @NonNull Gains<LinearVelocityUnit, DistanceUnit, LinearVelocityUnit> yGains,
      @NonNull LinearConstraints yConstraints,
      @NonNull Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit> rotationGains,
      @NonNull AngularPositionConstraints rotationConstraints,
      @NonNull Tunable<Distance> linearThreshold,
      @NonNull Tunable<Angle> angularThreshold) {}
}
