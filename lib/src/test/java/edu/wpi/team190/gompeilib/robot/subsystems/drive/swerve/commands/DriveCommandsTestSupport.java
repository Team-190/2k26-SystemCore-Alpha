package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.commands;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.TestGains;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDrive;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstants.AutoAlignConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveDriveConstantsTest;
import edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve.SwerveModuleConfig;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.tunable.Tunable;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;

/** Shared drive constants and a mocked drive for the swerve drive command tests. */
final class DriveCommandsTestSupport {
  static final double MAX_LINEAR = 4.0;
  static final double MAX_ANGULAR = 8.0;
  static final double DEADBAND = 0.1;
  static final double TOLERANCE = 1e-9;

  private DriveCommandsTestSupport() {}

  /** Rotation gains in radians per second per radian, published under the prefix. */
  static Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit> rotationGains(
      String prefix, double kP) {
    return Gains.builder(RadiansPerSecond, Radians, RadiansPerSecond)
        .withPrefix(prefix)
        .withKP(RadiansPerSecondPerRadian.ofNative(kP))
        .build();
  }

  static AutoAlignConstants autoAlign(
      Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit> rotationGains) {
    LinearConstraints linear =
        LinearConstraints.builder()
            .withMaxVelocity(MetersPerSecond.of(1.0))
            .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
            .withGoalTolerance(Meters.of(0.05))
            .build();
    return AutoAlignConstants.builder()
        .withXGains(SwerveDriveConstantsTest.translationGains(1.0, 0.0))
        .withXConstraints(linear)
        .withYGains(SwerveDriveConstantsTest.translationGains(1.0, 0.0))
        .withYConstraints(linear)
        .withRotationGains(rotationGains)
        .withRotationConstraints(
            AngularPositionConstraints.builder()
                .withMaxVelocity(RadiansPerSecond.of(4.0))
                .withMaxAcceleration(RadiansPerSecondPerSecond.of(8.0))
                .withGoalTolerance(Degrees.of(1.0))
                .build())
        .withLinearThreshold(Tunable.create(Meters.of(0.01)))
        .withAngularThreshold(Tunable.create(Radians.of(0.01)))
        .build();
  }

  static SwerveDriveConstants constants() {
    return constants(rotationGains("drive/rotation", 4.0));
  }

  static SwerveDriveConstants constants(
      Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit> rotationGains) {
    return constants(rotationGains, OutputType.VOLTAGE);
  }

  static SwerveDriveConstants constants(
      Gains<AngularVelocityUnit, AngleUnit, AngularVelocityUnit> rotationGains,
      OutputType driveOutput) {
    SwerveModuleConfig module = SwerveDriveConstantsTest.createModuleConfig(1, 2, 3, 0.5, 0.5);
    return SwerveDriveConstants.builder()
        .withDriveConfig(
            SwerveDriveConstants.DriveConfig.builder()
                .withCanBus(CANBusId.named("rio"))
                .withPigeon2Id(13)
                .withMaxLinearVelocityMetersPerSecond(MAX_LINEAR)
                .withWheelRadiusMeters(0.05)
                .withDriveModel(DCMotor.getKrakenX60(1))
                .withTurnModel(DCMotor.getKrakenX44(1))
                .withFrontLeft(module)
                .withFrontRight(module)
                .withBackLeft(module)
                .withBackRight(module)
                .withDriveClosedLoopOutputType(driveOutput)
                .withSteerClosedLoopOutputType(OutputType.VOLTAGE)
                .withBumperWidth(0.8)
                .withBumperLength(0.8)
                .withRobotMassKilograms(45.0)
                .withTrackWidth(0.6)
                .withRobotMOI(2.5)
                .withModuleCurrentLimit(40.0)
                .withWheelCOF(1.2)
                .build())
        .withDriveGains(TestGains.velocity(1.0, 0, 0, 0, 0, 0, 0))
        .withTurnGains(TestGains.position(1.0, 0, 0, 0, 0, 0, 0))
        .withAutoTranslationGains(SwerveDriveConstantsTest.translationGains(1.0, 0.0))
        .withAutoRotationGains(SwerveDriveConstantsTest.rotationGains(1.0, 0.0))
        .withAutoAlignConstants(autoAlign(rotationGains))
        .withOdometryFrequency(250.0)
        .withDriverDeadband(DEADBAND)
        .withOperatorDeadband(DEADBAND)
        .build();
  }

  /** A drive with fixed max speeds that measures no motion. */
  static SwerveDrive drive() {
    SwerveDrive drive = mock(SwerveDrive.class);
    when(drive.getMaxLinearSpeedMetersPerSec()).thenReturn(MAX_LINEAR);
    when(drive.getMaxAngularSpeedRadPerSec()).thenReturn(MAX_ANGULAR);
    when(drive.getMeasuredChassisVelocities()).thenReturn(new ChassisVelocities());
    return drive;
  }
}
