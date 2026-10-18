package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetry;
import edu.wpi.team190.gompeilib.io.gyro.interfaces.GyroIO;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Subsystem;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N2;
import org.wpilib.math.system.DCMotor;
import org.wpilib.tunable.Tunable;
import org.wpilib.units.measure.Angle;

@SuppressWarnings("unchecked") // Mockito mocks of generic types
public class SwerveDriveTest extends GompeiLibTestBase {

  private SwerveDriveConstants driveConstants;
  private SwerveModuleConfig moduleConfig;

  // The motors each module builds: drive first, then turn.
  private final List<MotorIO> flMotors = new ArrayList<>();

  private SwerveModuleHardware flModuleIO;
  private SwerveModuleHardware frModuleIO;
  private SwerveModuleHardware blModuleIO;
  private SwerveModuleHardware brModuleIO;

  private GyroIO lowFreqGyroIO;

  private Supplier<Pose2d> robotPoseSupplier;
  private Consumer<Pose2d> resetPoseConsumer;

  @BeforeEach
  public void setUp() {

    try {
      java.lang.reflect.Field instanceField =
          PhoenixOdometryThread.class.getDeclaredField("instance");
      instanceField.setAccessible(true);
      instanceField.set(null, null);
    } catch (Exception e) {
    }

    flMotors.clear();
    flModuleIO = moduleHardware(flMotors);
    frModuleIO = moduleHardware(new ArrayList<>());
    blModuleIO = moduleHardware(new ArrayList<>());
    brModuleIO = moduleHardware(new ArrayList<>());

    lowFreqGyroIO = mock(GyroIO.class);

    setupGyroIO(lowFreqGyroIO);

    robotPoseSupplier = () -> new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    resetPoseConsumer = pose -> {};

    CANBusId canBus = CANBusId.named("rio");
    DCMotor driveModel = DCMotor.getKrakenX60(1);
    DCMotor turnModel = DCMotor.getKrakenX44(1);

    moduleConfig = SwerveDriveConstantsTest.createModuleConfig(1, 2, 3, 0.5, 0.5);

    SwerveDriveConstants.DriveConfig driveConfig =
        SwerveDriveConstants.DriveConfig.builder()
            .withCanBus(canBus)
            .withPigeon2Id(13)
            .withMaxLinearVelocityMetersPerSecond(5.0)
            .withWheelRadiusMeters(0.05)
            .withDriveModel(driveModel)
            .withTurnModel(turnModel)
            .withFrontLeft(moduleConfig)
            .withFrontRight(moduleConfig)
            .withBackLeft(moduleConfig)
            .withBackRight(moduleConfig)
            .withDriveClosedLoopOutputType(OutputType.VOLTAGE)
            .withSteerClosedLoopOutputType(OutputType.VOLTAGE)
            .withBumperWidth(0.8)
            .withBumperLength(0.8)
            .withRobotMassKilograms(45.0)
            .withTrackWidth(0.6)
            .withRobotMOI(2.5)
            .withModuleCurrentLimit(40.0)
            .withWheelCOF(1.2)
            .build();

    Gains driveGains = Gains.builder().withKP(1.0).withKD(0.1).build();

    Gains turnGains = Gains.builder().withKP(2.0).withKD(0.2).build();

    SwerveDriveConstants.AutoAlignConstants autoAlignConstants =
        SwerveDriveConstants.AutoAlignConstants.builder()
            .withXGains(driveGains)
            .withXConstraints(
                LinearConstraints.builder()
                    .withMaxVelocity(MetersPerSecond.of(1.0))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Meters.of(0.05))
                    .build())
            .withYGains(driveGains)
            .withYConstraints(
                LinearConstraints.builder()
                    .withMaxVelocity(MetersPerSecond.of(1.0))
                    .withMaxAcceleration(MetersPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Meters.of(0.05))
                    .build())
            .withRotationGains(turnGains)
            .withRotationConstraints(
                AngularPositionConstraints.builder()
                    .withMaxVelocity(RadiansPerSecond.of(1.0))
                    .withMaxAcceleration(RadiansPerSecondPerSecond.of(1.0))
                    .withGoalTolerance(Radians.of(0.05))
                    .build())
            .withLinearThreshold(Tunable.create(Meters.of(0.01)))
            .withAngularThreshold(Tunable.create(Radians.of(0.01)))
            .build();

    driveConstants =
        SwerveDriveConstants.builder()
            .withDriveConfig(driveConfig)
            .withDriveGains(driveGains)
            .withTurnGains(turnGains)
            .withAutoTranslationGains(driveGains)
            .withAutoRotationGains(turnGains)
            .withAutoAlignConstants(autoAlignConstants)
            .withOdometryFrequency(250.0)
            .withDriverDeadband(0.1)
            .withOperatorDeadband(0.1)
            .build();
  }

  /** Builds module hardware whose motors and encoder report fixed readings. */
  private static SwerveModuleHardware moduleHardware(List<MotorIO> created) {
    MotorIOFactory factory = mock(MotorIOFactory.class);
    when(factory.family()).thenReturn("Mock");
    when(factory.create(any()))
        .thenAnswer(
            invocation -> {
              boolean isDrive = created.isEmpty();
              MotorIO motor = mock(MotorIO.class);
              doAnswer(
                      update -> {
                        MotorTelemetry t = update.getArgument(0);
                        t.connected = true;
                        t.position = isDrive ? Radians.of(10.0) : Degrees.of(30.0);
                        t.velocity = RadiansPerSecond.of(isDrive ? 5.0 : 0.0);
                        t.odometryTimestamps = new double[] {1.0};
                        t.odometryPositions = new Angle[] {t.position};
                        return null;
                      })
                  .when(motor)
                  .updateInputs(any());
              created.add(motor);
              return motor;
            });
    AbsoluteEncoderIO encoder = mock(AbsoluteEncoderIO.class);
    doAnswer(
            update -> {
              ((AbsoluteEncoderTelemetry) update.getArgument(0)).connected = true;
              return null;
            })
        .when(encoder)
        .updateInputs(any());
    return new SwerveModuleHardware(factory, factory, encoder);
  }

  private void setupGyroIO(GyroIO io) {
    doAnswer(
            invocation -> {
              GyroTelemetry in = invocation.getArgument(0);
              in.connected = true;
              in.yawPosition = Degrees.of(15.0);
              in.yawVelocity = RadiansPerSecond.of(0.5);
              in.odometryTimestamps = new double[] {1.0};
              in.odometryYawPositions = new Angle[] {Degrees.of(15.0)};
              return null;
            })
        .when(io)
        .updateInputs(any());
  }

  @Test
  public void testSwerveDriveLowFrequencyGyro() throws Exception {
    try (MockedStatic<AutoBuilder> mockAutoBuilder = mockStatic(AutoBuilder.class);
        MockedStatic<RobotConfig> mockRobotConfig = mockStatic(RobotConfig.class);
        MockedStatic<Logger> _ = mockStatic(Logger.class);
        MockedStatic<RobotState> mockRobotState = mockStatic(RobotState.class);
        MockedStatic<DriverStationBackend> mockDSBackend = mockStatic(DriverStationBackend.class)) {

      mockRobotConfig.when(RobotConfig::fromGUISettings).thenReturn(mock(RobotConfig.class));
      mockRobotState.when(RobotState::isDisabled).thenReturn(false);
      mockDSBackend.when(DriverStationBackend::getAlliance).thenReturn(Optional.of(Alliance.RED));

      // Capture the callbacks handed to AutoBuilder.configure and invoke them directly, since the
      // mocked AutoBuilder never calls them itself.
      mockAutoBuilder
          .when(
              () ->
                  AutoBuilder.configure(
                      any(),
                      any(),
                      any(),
                      any(BiConsumer.class),
                      any(),
                      any(),
                      any(),
                      any(Subsystem.class)))
          .thenAnswer(
              invocation -> {
                Supplier<ChassisVelocities> robotRelativeSpeedsSupplier = invocation.getArgument(2);
                BiConsumer<ChassisVelocities, DriveFeedforwards> output = invocation.getArgument(3);
                BooleanSupplier shouldFlipPath = invocation.getArgument(6);

                assertNotNull(robotRelativeSpeedsSupplier.get());
                output.accept(new ChassisVelocities(1.0, -1.0, 0.5), DriveFeedforwards.zeros(4));
                assertTrue(shouldFlipPath.getAsBoolean());

                // Also cover the present-but-not-RED case (alliance.get() == RED evaluates false).
                mockDSBackend
                    .when(DriverStationBackend::getAlliance)
                    .thenReturn(Optional.of(Alliance.BLUE));
                assertFalse(shouldFlipPath.getAsBoolean());
                mockDSBackend
                    .when(DriverStationBackend::getAlliance)
                    .thenReturn(Optional.of(Alliance.RED));

                return null;
              });

      SwerveDrive drive =
          new SwerveDrive(
              driveConstants,
              lowFreqGyroIO,
              flModuleIO,
              frModuleIO,
              blModuleIO,
              brModuleIO,
              robotPoseSupplier,
              resetPoseConsumer);

      // Verify simple getters
      assertEquals(5.0, drive.getMaxLinearSpeedMetersPerSec(), 1e-6);
      assertEquals(5.0 / Math.hypot(0.5, 0.5), drive.getMaxAngularSpeedRadPerSec(), 1e-6);

      // Periodic execution
      drive.periodic();

      assertEquals(0.5, drive.getYawVelocity(), 1e-6);

      // Verify modules read
      verify(flMotors.get(0), atLeastOnce()).updateInputs(any());

      // runVelocity
      ChassisVelocities targetSpeeds = new ChassisVelocities(1.0, -1.0, 0.5);
      drive.runVelocity(targetSpeeds);
      // atLeastOnce(): the mocked AutoBuilder.configure's captured output callback also invokes
      // runVelocity once during construction to cover that lambda.
      verify(flMotors.get(0), atLeastOnce()).setControl(isA(VelocityRequest.VelocityVoltage.class));

      // stop
      drive.stop();

      // stopWithX
      drive.stopWithX();

      // runVelocityTorque
      List<Vector<N2>> forces = new ArrayList<>();
      for (int i = 0; i < 4; i++) {
        forces.add(VecBuilder.fill(10.0, 5.0));
      }
      drive.runVelocityTorque(targetSpeeds, forces);

      // runCharacterization
      drive.runCharacterization(6.0);
      verify(flMotors.get(0), atLeastOnce())
          .setControl(new OpenLoopRequest.CurrentOutput(Amps.of(6.0)));

      // getModulePositions
      SwerveModulePosition[] positions = drive.getModulePositions();
      assertEquals(4, positions.length);
      assertEquals(10.0 * 0.05, positions[0].distance, 1e-6);

      // getWheelRadiusCharacterizationPositions
      double[] wheelRadPos = drive.getWheelRadiusCharacterizationPositions();
      assertEquals(4, wheelRadPos.length);
      assertEquals(10.0, wheelRadPos[0], 1e-6);

      // getFFCharacterizationVelocity
      assertEquals(5.0 / (2 * Math.PI), drive.getFFCharacterizationVelocity(), 1e-6);

      // setPIDGains
      drive.setPIDGains(2.0, 0.2, 3.0, 0.3);
      verify(flMotors.get(0))
          .updateGains(argThat(g -> g.getKP() == 2.0 && g.getKD() == 0.2), eq(GainSlot.ZERO));
      verify(flMotors.get(1))
          .updateGains(argThat(g -> g.getKP() == 3.0 && g.getKD() == 0.3), eq(GainSlot.ZERO));

      // setFFGains
      drive.setFFGains(0.1, 0.15);
      verify(flMotors.get(0))
          .updateGains(argThat(g -> g.getKS() == 0.1 && g.getKV() == 0.15), eq(GainSlot.ZERO));

      // updateCurrentLimits
      drive.updateCurrentLimits(40.0, 30.0);
      verify(flMotors.get(0)).updateCurrentLimits(new CurrentLimits(Amps.of(60.0), Amps.of(40.0)));
      verify(flMotors.get(1)).updateCurrentLimits(new CurrentLimits(Amps.of(60.0), Amps.of(30.0)));

      // setAutoControllers
      drive.setAutoControllers(driveConstants.driveGains, driveConstants.turnGains);

      var f = Gains.class.getDeclaredField("lastNotifiedValues");
      f.setAccessible(true);
      f.set(driveConstants.driveGains, null);
      f.set(driveConstants.turnGains, null);
      f.set(driveConstants.autoTranslationGains, null);
      f.set(driveConstants.autoRotationGains, null);
      var notifyGains = Gains.class.getDeclaredMethod("notifyListeners");
      notifyGains.setAccessible(true);
      notifyGains.invoke(driveConstants.driveGains);
      notifyGains.invoke(driveConstants.turnGains);
      notifyGains.invoke(driveConstants.autoTranslationGains);
      notifyGains.invoke(driveConstants.autoRotationGains);

      // TODO: restore choreoDrive test once ChoreoLib ships a WPILib 2027 alpha build.

      // getFieldRelativeVelocity
      Translation2d fieldRelVel = drive.getFieldRelativeVelocity();
      assertNotNull(fieldRelVel);
    }
  }

  @Test
  public void testSwerveDriveStartsTheOdometryThread() {
    try (MockedStatic<AutoBuilder> _ = mockStatic(AutoBuilder.class);
        MockedStatic<RobotConfig> mockRobotConfig = mockStatic(RobotConfig.class);
        MockedStatic<Logger> _ = mockStatic(Logger.class);
        MockedStatic<RobotState> mockRobotState = mockStatic(RobotState.class);
        MockedStatic<PhoenixOdometryThread> mockOdomThread =
            mockStatic(PhoenixOdometryThread.class)) {

      mockRobotConfig.when(RobotConfig::fromGUISettings).thenReturn(mock(RobotConfig.class));
      mockRobotState.when(RobotState::isDisabled).thenReturn(true); // Test isDisabled branch

      PhoenixOdometryThread mockThread = mock(PhoenixOdometryThread.class);
      mockOdomThread.when(() -> PhoenixOdometryThread.getInstance(any())).thenReturn(mockThread);

      SwerveDrive drive =
          new SwerveDrive(
              driveConstants,
              lowFreqGyroIO,
              flModuleIO,
              frModuleIO,
              blModuleIO,
              brModuleIO,
              robotPoseSupplier,
              resetPoseConsumer);

      verify(mockThread).start();
      drive.periodic();
      assertEquals(15.0, drive.getRawGyroRotation().getDegrees(), 1e-9);
    }
  }

  @Test
  public void testSwerveDriveUsesTheLatestYawWhenOdometrySamplesRunOut() {
    try (MockedStatic<AutoBuilder> _ = mockStatic(AutoBuilder.class);
        MockedStatic<RobotConfig> mockRobotConfig = mockStatic(RobotConfig.class);
        MockedStatic<Logger> _ = mockStatic(Logger.class);
        MockedStatic<RobotState> _ = mockStatic(RobotState.class)) {

      mockRobotConfig.when(RobotConfig::fromGUISettings).thenReturn(mock(RobotConfig.class));
      doAnswer(
              invocation -> {
                GyroTelemetry in = invocation.getArgument(0);
                in.connected = true;
                in.yawPosition = Degrees.of(42.0);
                in.odometryTimestamps = new double[] {};
                in.odometryYawPositions = new Angle[] {};
                return null;
              })
          .when(lowFreqGyroIO)
          .updateInputs(any());

      SwerveDrive drive =
          new SwerveDrive(
              driveConstants,
              lowFreqGyroIO,
              flModuleIO,
              frModuleIO,
              blModuleIO,
              brModuleIO,
              robotPoseSupplier,
              resetPoseConsumer);

      drive.periodic();
      assertEquals(42.0, drive.getRawGyroRotation().getDegrees(), 1e-9);
    }
  }

  @Test
  public void testSwerveDriveDisconnectedGyroFallback() {
    try (MockedStatic<AutoBuilder> mockAutoBuilder = mockStatic(AutoBuilder.class);
        MockedStatic<RobotConfig> mockRobotConfig = mockStatic(RobotConfig.class);
        MockedStatic<Logger> _ = mockStatic(Logger.class);
        MockedStatic<RobotState> _ = mockStatic(RobotState.class);
        MockedStatic<DriverStationBackend> mockDSBackend = mockStatic(DriverStationBackend.class)) {

      mockRobotConfig.when(RobotConfig::fromGUISettings).thenReturn(mock(RobotConfig.class));
      mockDSBackend.when(DriverStationBackend::getAlliance).thenReturn(Optional.empty());

      // Capture the shouldFlipPath callback to exercise the no-alliance-reported branch.
      mockAutoBuilder
          .when(
              () ->
                  AutoBuilder.configure(
                      any(),
                      any(),
                      any(),
                      any(BiConsumer.class),
                      any(),
                      any(),
                      any(),
                      any(Subsystem.class)))
          .thenAnswer(
              invocation -> {
                BooleanSupplier shouldFlipPath = invocation.getArgument(6);
                assertFalse(shouldFlipPath.getAsBoolean());
                return null;
              });

      // Make gyro report disconnected to test kinematics fallback in periodic
      doAnswer(
              invocation -> {
                GyroTelemetry in = invocation.getArgument(0);
                in.connected = false;
                in.odometryTimestamps = new double[] {1.0};
                in.odometryYawPositions = new Angle[] {Degrees.of(0.0)};
                return null;
              })
          .when(lowFreqGyroIO)
          .updateInputs(any());

      SwerveDrive drive =
          new SwerveDrive(
              driveConstants,
              lowFreqGyroIO,
              flModuleIO,
              frModuleIO,
              blModuleIO,
              brModuleIO,
              robotPoseSupplier,
              resetPoseConsumer);

      drive.periodic();
      assertNotNull(drive.getRawGyroRotation());
    }
  }

  @Test
  public void testSwerveDriveRobotConfigLoadFailure() {
    try (MockedStatic<AutoBuilder> _ = mockStatic(AutoBuilder.class);
        MockedStatic<RobotConfig> mockRobotConfig = mockStatic(RobotConfig.class);
        MockedStatic<Logger> _ = mockStatic(Logger.class);
        MockedStatic<RobotState> mockRobotState = mockStatic(RobotState.class)) {

      mockRobotConfig.when(RobotConfig::fromGUISettings).thenThrow(new RuntimeException("boom"));
      mockRobotState.when(RobotState::isDisabled).thenReturn(false);

      assertThrows(
          RuntimeException.class,
          () ->
              new SwerveDrive(
                  driveConstants,
                  lowFreqGyroIO,
                  flModuleIO,
                  frModuleIO,
                  blModuleIO,
                  brModuleIO,
                  robotPoseSupplier,
                  resetPoseConsumer));
    }
  }
}
