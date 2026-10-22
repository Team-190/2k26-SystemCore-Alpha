package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.data.VisionSingleTxTyObservation;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.CameraIOLimelight;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.LimelightIOInputsAutoLogged;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.limelight.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.driverstation.RobotState;
import org.wpilib.fields.Field;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;

public class CameraLimelightTest {

  @Test
  public void testCameraStaticLimelight() {
    CameraIOLimelight io = mock(CameraIOLimelight.class);
    VisionConstants.StaticLimelightConfig config =
        VisionConstants.StaticLimelightConfig.builder()
            .key("static")
            .cameraType(CameraType.LIMELIGHT_3)
            .horizontalFOV(60.0)
            .verticalFOV(40.0)
            .megatagXYStdev(0.1)
            .metatagThetaStdev(0.2)
            .megatag2XYStdev(0.05)
            .robotToCameraTransform(
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d()))
            .enableRewind(true)
            .build();

    List<VisionPoseObservation> poseObs = new ArrayList<>();
    List<VisionSingleTxTyObservation> singleObs = new ArrayList<>();

    Consumer<List<VisionPoseObservation>> poseObserver = poseObs::addAll;
    Consumer<List<VisionSingleTxTyObservation>> singleObserver = singleObs::addAll;

    ChassisVelocities[] staticChassisVelocities = {new ChassisVelocities()};

    try (MockedStatic<LimelightHelpers> mockHelpers = mockStatic(LimelightHelpers.class);
        MockedStatic<RobotState> mockDS = mockStatic(RobotState.class);
        MockedStatic<Timer> mockTimer = mockStatic(Timer.class);
        MockedStatic<GompeiLib> mockLib = mockStatic(GompeiLib.class);
        var _ = mockStatic(Logger.class)) {

      mockDS.when(RobotState::isEnabled).thenReturn(false);
      mockDS.when(RobotState::isDisabled).thenReturn(true);
      mockTimer.when(Timer::getTimestamp).thenReturn(10.0);

      // Construct once while tuning (under a distinct key, so it doesn't affect the
      // "limelight-static" invocation counts verified below), to cover the constructor's tuning
      // branch
      mockLib.when(GompeiLib::isTuning).thenReturn(true);
      VisionConstants.StaticLimelightConfig tuningProbeConfig =
          VisionConstants.StaticLimelightConfig.builder()
              .key("static-tuning-probe")
              .cameraType(CameraType.LIMELIGHT_3)
              .horizontalFOV(60.0)
              .verticalFOV(40.0)
              .megatagXYStdev(0.1)
              .metatagThetaStdev(0.2)
              .megatag2XYStdev(0.05)
              .robotToCameraTransform(
                  new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d()))
              .enableRewind(true)
              .build();
      new CameraStaticLimelight(
          mock(CameraIOLimelight.class),
          tuningProbeConfig,
          () -> Rotation2d.fromDegrees(0),
          () -> new ChassisVelocities(),
          () -> 1000L,
          List.of(poseObserver),
          List.of(singleObserver));

      mockLib.when(GompeiLib::isTuning).thenReturn(false);

      CameraStaticLimelight camera =
          new CameraStaticLimelight(
              io,
              config,
              () -> Rotation2d.fromDegrees(0),
              () -> staticChassisVelocities[0],
              () -> 1000L,
              List.of(poseObserver),
              List.of(singleObserver));

      assertEquals("limelight-static", camera.getName());

      double[] staticTimestamp = {10.0};

      // --- Setup input data ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .LimelightIOInputsAutoLogged)
                        invocation.getArgument(0);

                double ts = staticTimestamp[0];
                staticTimestamp[0] += 0.02;

                // Setup MT1 Pose Estimate
                CameraIO.RawFiducial fid =
                    new CameraIO.RawFiducial(1, 0.1, 0.2, 0.3, 2.0, 2.0, 0.05);
                inputs.mt1PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.0, 2.0, new Rotation2d()),
                        ts,
                        5.0,
                        1,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {fid},
                        false);

                // Setup MT2 Pose Estimate
                inputs.mt2PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.1, 2.1, new Rotation2d()),
                        ts,
                        5.0,
                        1,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {fid},
                        true);

                // Setup raw fiducials
                inputs.rawFiducials = new CameraIO.RawFiducial[] {fid};

                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      // Periodic check while disabled
      camera.periodic();
      assertFalse(poseObs.isEmpty());
      assertEquals(1, poseObs.size()); // MT2 XY only (MT1 single tag doesn't pass theta gate)
      assertEquals(1, singleObs.size());

      // Transition to enabled
      mockDS.when(RobotState::isEnabled).thenReturn(true);
      mockDS.when(RobotState::isDisabled).thenReturn(false);
      mockTimer.when(Timer::getTimestamp).thenReturn(11.0);

      poseObs.clear();
      singleObs.clear();
      camera.periodic();

      mockHelpers.verify(() -> LimelightHelpers.SetIMUMode("limelight-static", 0), times(1));
      mockHelpers.verify(() -> LimelightHelpers.SetThrottle("limelight-static", 0), times(1));

      // Rerun enabled periodic with time advance to trigger rewind
      mockTimer.when(Timer::getTimestamp).thenReturn(180.0); // > 165
      camera.periodic();
      mockHelpers.verify(
          () -> LimelightHelpers.triggerRewindCapture("limelight-static", 165), times(1));

      // Transition to disabled while tuning
      mockDS.when(RobotState::isEnabled).thenReturn(false);
      mockDS.when(RobotState::isDisabled).thenReturn(true);
      mockLib.when(GompeiLib::isTuning).thenReturn(true);
      mockTimer.when(Timer::getTimestamp).thenReturn(185.0);

      camera.periodic();
      mockHelpers.verify(
          () -> LimelightHelpers.SetIMUMode("limelight-static", 1),
          times(2)); // First in constructor/constructor states

      // Transition back to enabled, then to disabled while not tuning, to cover the
      // periodic() disabled-block's non-tuning branch
      mockDS.when(RobotState::isEnabled).thenReturn(true);
      mockDS.when(RobotState::isDisabled).thenReturn(false);
      mockTimer.when(Timer::getTimestamp).thenReturn(200.0);
      camera.periodic();

      mockDS.when(RobotState::isEnabled).thenReturn(false);
      mockDS.when(RobotState::isDisabled).thenReturn(true);
      mockLib.when(GompeiLib::isTuning).thenReturn(false);
      mockTimer.when(Timer::getTimestamp).thenReturn(201.0);
      camera.periodic();
      // Once from the constructor (also built with tuning=false), once from this periodic() call.
      mockHelpers.verify(() -> LimelightHelpers.SetThrottle("limelight-static", 190), times(2));

      // Multiple confident tags with a stationary chassis: covers the high-confidence
      // theta stdev branch (as opposed to the low-confidence Double.POSITIVE_INFINITY fallback).
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .LimelightIOInputsAutoLogged)
                        invocation.getArgument(0);
                CameraIO.RawFiducial fid1 =
                    new CameraIO.RawFiducial(1, 0.1, 0.2, 0.3, 2.0, 2.0, 0.05);
                CameraIO.RawFiducial fid2 =
                    new CameraIO.RawFiducial(2, 0.1, 0.2, 0.3, 2.0, 2.0, 0.05);
                double ts = staticTimestamp[0];
                staticTimestamp[0] += 0.02;
                inputs.mt1PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.0, 2.0, new Rotation2d()),
                        ts,
                        5.0,
                        2,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {fid1, fid2},
                        false);
                inputs.mt2PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.1, 2.1, new Rotation2d()),
                        ts,
                        5.0,
                        0,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {},
                        true);
                inputs.rawFiducials = new CameraIO.RawFiducial[] {fid1, fid2};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      poseObs.clear();
      camera.periodic();
      assertEquals(1, poseObs.size());
      assertTrue(poseObs.get(0).stddevs().get(2, 0) < Double.POSITIVE_INFINITY);

      // Excess vx, vy, and omega each independently fall back to POSITIVE_INFINITY (still using
      // the tagCount=2 confident-fiducial stub from above).
      staticChassisVelocities[0] = new ChassisVelocities(1.0, 0.0, 0.0);
      poseObs.clear();
      camera.periodic();
      assertEquals(Double.POSITIVE_INFINITY, poseObs.get(0).stddevs().get(2, 0));

      staticChassisVelocities[0] = new ChassisVelocities(0.0, 1.0, 0.0);
      poseObs.clear();
      camera.periodic();
      assertEquals(Double.POSITIVE_INFINITY, poseObs.get(0).stddevs().get(2, 0));

      staticChassisVelocities[0] = new ChassisVelocities(0.0, 0.0, 1.0);
      poseObs.clear();
      camera.periodic();
      assertEquals(Double.POSITIVE_INFINITY, poseObs.get(0).stddevs().get(2, 0));

      staticChassisVelocities[0] = new ChassisVelocities();

      // Moving camera: High-ambiguity fiducials also fall back to POSITIVE_INFINITY.
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .LimelightIOInputsAutoLogged)
                        invocation.getArgument(0);
                CameraIO.RawFiducial ambiguousFid1 =
                    new CameraIO.RawFiducial(1, 0.1, 0.2, 0.3, 2.0, 2.0, 0.9);
                CameraIO.RawFiducial ambiguousFid2 =
                    new CameraIO.RawFiducial(2, 0.1, 0.2, 0.3, 2.0, 2.0, 0.9);
                double ts = staticTimestamp[0];
                staticTimestamp[0] += 0.02;
                inputs.mt1PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.0, 2.0, new Rotation2d()),
                        ts,
                        5.0,
                        2,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {ambiguousFid1, ambiguousFid2},
                        false);
                inputs.mt2PoseEstimate = new CameraIO.PoseEstimate();
                inputs.rawFiducials = new CameraIO.RawFiducial[] {};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      poseObs.clear();
      singleObs.clear();
      camera.periodic();
      assertEquals(1, poseObs.size());
      assertEquals(Double.POSITIVE_INFINITY, poseObs.get(0).stddevs().get(2, 0));
      // Also covers the mt1 tagCount==0/mt2 tagCount==0/rawFiducials-empty false branches.
      assertTrue(singleObs.isEmpty());

      // A separate camera built with rewind disabled, to cover the enableRewind()==false
      // branches in both the enabled-elapsed-time check and the disabled-transition check.
      VisionConstants.StaticLimelightConfig noRewindConfig =
          VisionConstants.StaticLimelightConfig.builder()
              .key("static-no-rewind")
              .cameraType(CameraType.LIMELIGHT_3)
              .horizontalFOV(60.0)
              .verticalFOV(40.0)
              .megatagXYStdev(0.1)
              .metatagThetaStdev(0.2)
              .megatag2XYStdev(0.05)
              .robotToCameraTransform(
                  new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d()))
              .enableRewind(false)
              .build();
      CameraStaticLimelight noRewindCamera =
          new CameraStaticLimelight(
              mock(CameraIOLimelight.class),
              noRewindConfig,
              () -> Rotation2d.fromDegrees(0),
              () -> new ChassisVelocities(),
              () -> 1000L,
              List.of(poseObserver),
              List.of(singleObserver));

      mockDS.when(RobotState::isEnabled).thenReturn(true);
      mockDS.when(RobotState::isDisabled).thenReturn(false);
      mockTimer.when(Timer::getTimestamp).thenReturn(300.0);
      noRewindCamera.periodic();

      mockTimer.when(Timer::getTimestamp).thenReturn(500.0); // elapsed > 165
      noRewindCamera.periodic();
      mockHelpers.verify(
          () -> LimelightHelpers.triggerRewindCapture(eq("static-no-rewind"), anyDouble()),
          never());

      mockDS.when(RobotState::isEnabled).thenReturn(false);
      mockDS.when(RobotState::isDisabled).thenReturn(true);
      noRewindCamera.periodic();
      mockHelpers.verify(
          () -> LimelightHelpers.triggerRewindCapture(eq("static-no-rewind"), anyDouble()),
          never());
    }
  }

  @Test
  public void testCameraMovingLimelight() {
    CameraIOLimelight io = mock(CameraIOLimelight.class);
    VisionConstants.MovingLimelightConfig config =
        VisionConstants.MovingLimelightConfig.builder()
            .key("moving")
            .cameraType(CameraType.LIMELIGHT_3G)
            .horizontalFOV(80.0)
            .verticalFOV(50.0)
            .megatagXYStdev(0.1)
            .metatagThetaStdev(0.2)
            .megatag2XYStdev(0.05)
            .robotToRotationAxisTransform(
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d()))
            .rotationAxisToLensTransform(
                new Transform3d(new Translation3d(0.1, 0.0, 0.0), new Rotation3d()))
            .enableRewind(true)
            .build();

    List<VisionPoseObservation> poseObs = new ArrayList<>();
    List<VisionSingleTxTyObservation> singleObs = new ArrayList<>();

    Consumer<List<VisionPoseObservation>> poseObserver = poseObs::addAll;
    Consumer<List<VisionSingleTxTyObservation>> singleObserver = singleObs::addAll;

    ChassisVelocities[] movingChassisVelocities = {new ChassisVelocities()};

    try (MockedStatic<LimelightHelpers> mockHelpers = mockStatic(LimelightHelpers.class);
        MockedStatic<RobotState> mockDS = mockStatic(RobotState.class);
        MockedStatic<Timer> mockTimer = mockStatic(Timer.class);
        var _ = mockStatic(Logger.class)) {

      mockDS.when(RobotState::isEnabled).thenReturn(false);
      mockDS.when(RobotState::isDisabled).thenReturn(true);
      mockTimer.when(Timer::getTimestamp).thenReturn(10.0);

      CameraMovingLimelight camera =
          new CameraMovingLimelight(
              io,
              config,
              () -> Rotation2d.fromDegrees(0),
              () -> Rotation2d.fromDegrees(45), // rotated 45 degrees
              () -> movingChassisVelocities[0],
              () -> 1000L,
              List.of(poseObserver),
              List.of(singleObserver));

      assertEquals("limelight-moving", camera.getName());

      double[] movingTimestamp = {10.0};

      // --- Setup input data ---
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .LimelightIOInputsAutoLogged)
                        invocation.getArgument(0);

                double ts = movingTimestamp[0];
                movingTimestamp[0] += 0.02;

                CameraIO.RawFiducial fid =
                    new CameraIO.RawFiducial(1, 0.1, 0.2, 0.3, 2.0, 2.0, 0.05);
                inputs.mt1PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.0, 2.0, new Rotation2d()),
                        ts,
                        5.0,
                        1,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {fid},
                        false);
                inputs.mt2PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.1, 2.1, new Rotation2d()),
                        ts,
                        5.0,
                        1,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {fid},
                        true);
                inputs.rawFiducials = new CameraIO.RawFiducial[] {fid};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      camera.periodic();
      assertFalse(poseObs.isEmpty());
      assertEquals(1, poseObs.size()); // MT2 XY only (single tag does not pass theta gate)

      // Transition to enabled
      mockDS.when(RobotState::isEnabled).thenReturn(true);
      mockDS.when(RobotState::isDisabled).thenReturn(false);
      mockTimer.when(Timer::getTimestamp).thenReturn(11.0);

      camera.periodic();
      mockHelpers.verify(() -> LimelightHelpers.SetIMUMode("limelight-moving", 0), times(1));
      mockHelpers.verify(() -> LimelightHelpers.SetThrottle("limelight-moving", 0), times(1));

      // Advance time to trigger rewind capture
      mockTimer.when(Timer::getTimestamp).thenReturn(180.0);
      camera.periodic();
      mockHelpers.verify(
          () -> LimelightHelpers.triggerRewindCapture("limelight-moving", 165), times(1));

      // Transition back to disabled
      mockDS.when(RobotState::isEnabled).thenReturn(false);
      mockDS.when(RobotState::isDisabled).thenReturn(true);
      mockTimer.when(Timer::getTimestamp).thenReturn(185.0);

      camera.periodic();
      mockHelpers.verify(() -> LimelightHelpers.SetIMUMode("limelight-moving", 1), times(2));

      // Multiple confident tags with a stationary chassis: covers the high-confidence
      // theta stdev branch (as opposed to the low-confidence Double.POSITIVE_INFINITY fallback).
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .LimelightIOInputsAutoLogged)
                        invocation.getArgument(0);
                CameraIO.RawFiducial fid1 =
                    new CameraIO.RawFiducial(1, 0.1, 0.2, 0.3, 2.0, 2.0, 0.05);
                CameraIO.RawFiducial fid2 =
                    new CameraIO.RawFiducial(2, 0.1, 0.2, 0.3, 2.0, 2.0, 0.05);
                double ts = movingTimestamp[0];
                movingTimestamp[0] += 0.02;
                inputs.mt1PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.0, 2.0, new Rotation2d()),
                        ts,
                        5.0,
                        2,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {fid1, fid2},
                        false);
                inputs.mt2PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.1, 2.1, new Rotation2d()),
                        ts,
                        5.0,
                        0,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {},
                        true);
                inputs.rawFiducials = new CameraIO.RawFiducial[] {fid1, fid2};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      poseObs.clear();
      camera.periodic();
      assertEquals(1, poseObs.size());
      assertTrue(poseObs.get(0).stddevs().get(2, 0) < Double.POSITIVE_INFINITY);

      // Excess vx, vy, and omega each independently fall back to POSITIVE_INFINITY (still using
      // the tagCount=2 confident-fiducial stub from above).
      movingChassisVelocities[0] = new ChassisVelocities(1.0, 0.0, 0.0);
      poseObs.clear();
      camera.periodic();
      assertEquals(Double.POSITIVE_INFINITY, poseObs.get(0).stddevs().get(2, 0));

      movingChassisVelocities[0] = new ChassisVelocities(0.0, 1.0, 0.0);
      poseObs.clear();
      camera.periodic();
      assertEquals(Double.POSITIVE_INFINITY, poseObs.get(0).stddevs().get(2, 0));

      movingChassisVelocities[0] = new ChassisVelocities(0.0, 0.0, 1.0);
      poseObs.clear();
      camera.periodic();
      assertEquals(Double.POSITIVE_INFINITY, poseObs.get(0).stddevs().get(2, 0));

      movingChassisVelocities[0] = new ChassisVelocities();

      // Moving camera: High-ambiguity fiducials also fall back to POSITIVE_INFINITY.
      doAnswer(
              invocation -> {
                var inputs =
                    (edu.wpi.team190.gompeilib.robot.subsystems.vision.io
                            .LimelightIOInputsAutoLogged)
                        invocation.getArgument(0);
                CameraIO.RawFiducial ambiguousFid1 =
                    new CameraIO.RawFiducial(1, 0.1, 0.2, 0.3, 2.0, 2.0, 0.9);
                CameraIO.RawFiducial ambiguousFid2 =
                    new CameraIO.RawFiducial(2, 0.1, 0.2, 0.3, 2.0, 2.0, 0.9);
                double ts = movingTimestamp[0];
                movingTimestamp[0] += 0.02;
                inputs.mt1PoseEstimate =
                    new CameraIO.PoseEstimate(
                        new Pose2d(1.0, 2.0, new Rotation2d()),
                        ts,
                        5.0,
                        2,
                        1.0,
                        2.0,
                        0.5,
                        new CameraIO.RawFiducial[] {ambiguousFid1, ambiguousFid2},
                        false);
                inputs.mt2PoseEstimate = new CameraIO.PoseEstimate();
                inputs.rawFiducials = new CameraIO.RawFiducial[] {};
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      poseObs.clear();
      singleObs.clear();
      camera.periodic();
      assertEquals(1, poseObs.size());
      assertEquals(Double.POSITIVE_INFINITY, poseObs.get(0).stddevs().get(2, 0));
      // Also covers the mt1 tagCount==0/mt2 tagCount==0/rawFiducials-empty false branches.
      assertTrue(singleObs.isEmpty());

      // A separate camera built with rewind disabled, to cover the enableRewind()==false
      // branches in both the enabled-elapsed-time check and the disabled-transition check.
      VisionConstants.MovingLimelightConfig noRewindConfig =
          VisionConstants.MovingLimelightConfig.builder()
              .key("moving-no-rewind")
              .cameraType(CameraType.LIMELIGHT_3G)
              .horizontalFOV(80.0)
              .verticalFOV(50.0)
              .megatagXYStdev(0.1)
              .metatagThetaStdev(0.2)
              .megatag2XYStdev(0.05)
              .robotToRotationAxisTransform(
                  new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d()))
              .rotationAxisToLensTransform(
                  new Transform3d(new Translation3d(0.1, 0.0, 0.0), new Rotation3d()))
              .enableRewind(false)
              .build();
      CameraMovingLimelight noRewindCamera =
          new CameraMovingLimelight(
              mock(CameraIOLimelight.class),
              noRewindConfig,
              () -> Rotation2d.fromDegrees(0),
              () -> Rotation2d.fromDegrees(45),
              () -> new ChassisVelocities(),
              () -> 1000L,
              List.of(poseObserver),
              List.of(singleObserver));

      mockDS.when(RobotState::isEnabled).thenReturn(true);
      mockDS.when(RobotState::isDisabled).thenReturn(false);
      mockTimer.when(Timer::getTimestamp).thenReturn(300.0);
      noRewindCamera.periodic();

      mockTimer.when(Timer::getTimestamp).thenReturn(500.0); // elapsed > 165
      noRewindCamera.periodic();
      mockHelpers.verify(
          () -> LimelightHelpers.triggerRewindCapture(eq("moving-no-rewind"), anyDouble()),
          never());

      mockDS.when(RobotState::isEnabled).thenReturn(false);
      mockDS.when(RobotState::isDisabled).thenReturn(true);
      noRewindCamera.periodic();
      mockHelpers.verify(
          () -> LimelightHelpers.triggerRewindCapture(eq("moving-no-rewind"), anyDouble()),
          never());
    }
  }

  @Test
  public void testStaleFrameDeduplication() {
    CameraIOLimelight io = mock(CameraIOLimelight.class);
    VisionConstants.StaticLimelightConfig config =
        VisionConstants.StaticLimelightConfig.builder()
            .key("static-stale")
            .cameraType(CameraType.LIMELIGHT_3)
            .horizontalFOV(60.0)
            .verticalFOV(40.0)
            .megatagXYStdev(0.1)
            .metatagThetaStdev(0.2)
            .megatag2XYStdev(0.05)
            .robotToCameraTransform(new Transform3d())
            .enableRewind(false)
            .build();

    List<VisionPoseObservation> poseObs = new ArrayList<>();
    LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();

    try (MockedStatic<LimelightHelpers> _ = mockStatic(LimelightHelpers.class);
        MockedStatic<Logger> _ = mockStatic(Logger.class);
        MockedStatic<RobotState> mockDS = mockStatic(RobotState.class);
        MockedStatic<Timer> mockTimer = mockStatic(Timer.class);
        MockedStatic<GompeiLib> mockLib = mockStatic(GompeiLib.class)) {
      mockDS.when(RobotState::isEnabled).thenReturn(false);
      mockDS.when(RobotState::isDisabled).thenReturn(true);
      mockTimer.when(Timer::getTimestamp).thenReturn(10.0);
      mockLib.when(GompeiLib::isTuning).thenReturn(false);

      CameraStaticLimelight camera =
          new CameraStaticLimelight(
              io,
              config,
              () -> Rotation2d.fromDegrees(0),
              () -> new ChassisVelocities(),
              () -> 1000L,
              List.of(poseObs::addAll),
              List.of());

      CameraIO.RawFiducial fid = new CameraIO.RawFiducial(1, 0.1, 0.2, 0.3, 2.0, 2.0, 0.05);
      inputs.mt2PoseEstimate =
          new CameraIO.PoseEstimate(
              new Pose2d(1.0, 2.0, new Rotation2d()),
              15.0,
              5.0,
              1,
              1.0,
              2.0,
              0.5,
              new CameraIO.RawFiducial[] {fid},
              true);
      inputs.rawFiducials = new CameraIO.RawFiducial[] {fid};

      doAnswer(
              invocation -> {
                var in = (LimelightIOInputsAutoLogged) invocation.getArgument(0);
                in.mt2PoseEstimate = inputs.mt2PoseEstimate;
                in.rawFiducials = inputs.rawFiducials;
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      // Loop 1: Fresh frame at timestamp 15.0 -> accepted
      camera.periodic();
      assertEquals(1, poseObs.size());

      // Loop 2: Stale frame at same timestamp 15.0 -> dropped!
      poseObs.clear();
      camera.periodic();
      assertEquals(0, poseObs.size());

      // Loop 3: Still stale frame at 15.0 -> dropped!
      camera.periodic();
      assertEquals(0, poseObs.size());

      // Loop 4: New fresh frame at timestamp 15.02 -> accepted!
      inputs.mt2PoseEstimate =
          new CameraIO.PoseEstimate(
              new Pose2d(1.0, 2.0, new Rotation2d()),
              15.02,
              5.0,
              1,
              1.0,
              2.0,
              0.5,
              new CameraIO.RawFiducial[] {fid},
              true);
      camera.periodic();
      assertEquals(1, poseObs.size());
    }
  }

  @Test
  public void testTurretCorrectionAtKnownAnglesAndPivotStability() {
    CameraIOLimelight io = mock(CameraIOLimelight.class);
    // Robot center at (0,0). Rotation axis at (0.5, 0). Axis to lens is (0.2, 0).
    VisionConstants.MovingLimelightConfig config =
        VisionConstants.MovingLimelightConfig.builder()
            .key("moving-turret-geom")
            .cameraType(CameraType.LIMELIGHT_3G)
            .horizontalFOV(80.0)
            .verticalFOV(50.0)
            .megatagXYStdev(0.1)
            .metatagThetaStdev(0.2)
            .megatag2XYStdev(0.05)
            .robotToRotationAxisTransform(
                new Transform3d(new Translation3d(0.5, 0.0, 0.0), new Rotation3d()))
            .rotationAxisToLensTransform(
                new Transform3d(new Translation3d(0.2, 0.0, 0.0), new Rotation3d()))
            .enableRewind(false)
            .build();

    List<VisionPoseObservation> poseObs = new ArrayList<>();
    Rotation2d[] turretAngle = {Rotation2d.fromDegrees(0)};
    double[] timestamp = {10.0};
    LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();

    try (MockedStatic<LimelightHelpers> _ = mockStatic(LimelightHelpers.class);
        MockedStatic<Logger> _ = mockStatic(Logger.class);
        MockedStatic<Timer> mockTimer = mockStatic(Timer.class)) {
      mockTimer.when(Timer::getTimestamp).thenAnswer(_ -> timestamp[0]);

      CameraMovingLimelight camera =
          new CameraMovingLimelight(
              io,
              config,
              () -> Rotation2d.fromDegrees(0),
              () -> turretAngle[0],
              () -> new ChassisVelocities(),
              () -> 1000L,
              List.of(poseObs::addAll),
              List.of());

      doAnswer(
              inv -> {
                var in = (LimelightIOInputsAutoLogged) inv.getArgument(0);
                in.mt2PoseEstimate = inputs.mt2PoseEstimate;
                in.mt1PoseEstimate = inputs.mt1PoseEstimate;
                in.rawFiducials = inputs.rawFiducials;
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      CameraIO.RawFiducial fid = new CameraIO.RawFiducial(1, 0.0, 0.0, 1.0, 2.0, 2.0, 0.05);
      inputs.rawFiducials = new CameraIO.RawFiducial[] {fid};

      // Limelight was initialized believing lens is at (0.5, 0).
      // If true robot is at (0, 0, 0):
      // True camera field pose at 0 deg turret: (0.7, 0, 0).
      // Limelight sees camera at (0.7, 0, 0), believes camera offset is (0.5, 0), so LL reports
      // robot at (0.2, 0, 0).
      // Turret correction should recover true robot at (0, 0, 0)!

      // 1) Turret at 0 deg
      turretAngle[0] = Rotation2d.fromDegrees(0);
      timestamp[0] = 10.0;
      inputs.mt2PoseEstimate =
          new CameraIO.PoseEstimate(
              new Pose2d(0.2, 0.0, new Rotation2d()),
              timestamp[0],
              5.0,
              1,
              1.0,
              2.0,
              0.5,
              new CameraIO.RawFiducial[] {fid},
              true);

      camera.periodic();
      assertEquals(1, poseObs.size());
      assertEquals(0.0, poseObs.get(0).pose().getX(), 1e-4);
      assertEquals(0.0, poseObs.get(0).pose().getY(), 1e-4);
      assertEquals(0.7, camera.getCurrentCameraPose().getX(), 1e-4);
      assertEquals(0.0, camera.getCurrentCameraPose().getY(), 1e-4);

      // Verify pivot does NOT drift over subsequent loops
      timestamp[0] = 10.02;
      inputs.mt2PoseEstimate =
          new CameraIO.PoseEstimate(
              new Pose2d(0.2, 0.0, new Rotation2d()),
              timestamp[0],
              5.0,
              1,
              1.0,
              2.0,
              0.5,
              new CameraIO.RawFiducial[] {fid},
              true);
      poseObs.clear();
      camera.periodic();
      assertEquals(0.7, camera.getCurrentCameraPose().getX(), 1e-4);
      assertEquals(0.0, camera.getCurrentCameraPose().getY(), 1e-4);

      // 2) Turret at 90 deg:
      // Lens rotated 90 deg around axis (0.5, 0):
      // True lens position is (0.5, 0.2, 0), yaw = 90 deg.
      // Believed camera is (0.5, 0, 0).
      // If true robot is at (0, 0, 0), LL reports camera at (0.5, 0.2, 90 deg).
      // Relative to believed (0.5, 0), LL reports believed robot at (0.5 - 0.5*cos(90) + ..., ...)
      // With our exact transform formula: trueRobot = believedRobot ∘ robotToBelievedCam ∘
      // robotToTrueCam⁻¹
      // So if believedRobot ∘ robotToBelievedCam = trueCam, then trueCam ∘ robotToTrueCam⁻¹ =
      // trueRobot = (0,0)!
      turretAngle[0] = Rotation2d.fromDegrees(90);
      timestamp[0] = 10.04;
      Pose2d trueCam90 = new Pose2d(0.5, 0.2, Rotation2d.fromDegrees(90));
      Pose2d believedCam = new Pose2d(0.5, 0.0, Rotation2d.fromDegrees(0));
      Pose2d believedRobot90 =
          trueCam90.transformBy(
              new org.wpilib.math.geometry.Transform2d(Pose2d.ZERO, believedCam).inverse());
      inputs.mt2PoseEstimate =
          new CameraIO.PoseEstimate(
              believedRobot90,
              timestamp[0],
              5.0,
              1,
              1.0,
              2.0,
              0.5,
              new CameraIO.RawFiducial[] {fid},
              true);

      poseObs.clear();
      camera.periodic();
      assertEquals(1, poseObs.size());
      assertEquals(0.0, poseObs.get(0).pose().getX(), 1e-4);
      assertEquals(0.0, poseObs.get(0).pose().getY(), 1e-4);
      assertEquals(0.5, camera.getCurrentCameraPose().getX(), 1e-4);
      assertEquals(0.2, camera.getCurrentCameraPose().getY(), 1e-4);
      assertEquals(
          90.0, camera.getCurrentCameraPose().getRotation().getZ() * 180.0 / Math.PI, 1e-4);

      // 3) Turret at 180 deg:
      // Lens rotated 180 deg around axis (0.5, 0):
      // True lens position is (0.3, 0.0, 0), yaw = 180 deg.
      turretAngle[0] = Rotation2d.fromDegrees(180);
      timestamp[0] = 10.06;
      Pose2d trueCam180 = new Pose2d(0.3, 0.0, Rotation2d.fromDegrees(180));
      Pose2d believedRobot180 =
          trueCam180.transformBy(
              new org.wpilib.math.geometry.Transform2d(Pose2d.ZERO, believedCam).inverse());
      inputs.mt2PoseEstimate =
          new CameraIO.PoseEstimate(
              believedRobot180,
              timestamp[0],
              5.0,
              1,
              1.0,
              2.0,
              0.5,
              new CameraIO.RawFiducial[] {fid},
              true);

      poseObs.clear();
      camera.periodic();
      assertEquals(1, poseObs.size());
      assertEquals(0.0, poseObs.get(0).pose().getX(), 1e-4);
      assertEquals(0.0, poseObs.get(0).pose().getY(), 1e-4);
      assertEquals(0.3, camera.getCurrentCameraPose().getX(), 1e-4);
      assertEquals(0.0, camera.getCurrentCameraPose().getY(), 1e-4);
      assertEquals(
          180.0,
          Math.abs(camera.getCurrentCameraPose().getRotation().getZ() * 180.0 / Math.PI),
          1e-4);
    }
  }

  @Test
  public void testLimelightRejectionFilters() {
    CameraIOLimelight io = mock(CameraIOLimelight.class);
    VisionConstants.StaticLimelightConfig config =
        VisionConstants.StaticLimelightConfig.builder()
            .key("static-filter")
            .cameraType(CameraType.LIMELIGHT_3)
            .horizontalFOV(60.0)
            .verticalFOV(40.0)
            .megatagXYStdev(0.1)
            .metatagThetaStdev(0.2)
            .megatag2XYStdev(0.05)
            .robotToCameraTransform(new Transform3d())
            .enableRewind(false)
            .build();

    List<VisionPoseObservation> poseObs = new ArrayList<>();
    ChassisVelocities[] chassisVelocities = {new ChassisVelocities()};
    double[] timestamp = {1.0};
    LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();

    try (MockedStatic<LimelightHelpers> _ = mockStatic(LimelightHelpers.class);
        MockedStatic<Logger> _ = mockStatic(Logger.class);
        MockedStatic<RobotState> mockDS = mockStatic(RobotState.class);
        MockedStatic<Timer> mockTimer = mockStatic(Timer.class);
        MockedStatic<GompeiLib> mockLib = mockStatic(GompeiLib.class)) {
      mockDS.when(RobotState::isEnabled).thenReturn(false);
      mockDS.when(RobotState::isDisabled).thenReturn(true);
      mockTimer.when(Timer::getTimestamp).thenAnswer(_ -> timestamp[0]);
      mockLib.when(GompeiLib::isTuning).thenReturn(false);

      CameraStaticLimelight camera =
          new CameraStaticLimelight(
              io,
              config,
              () -> Rotation2d.fromDegrees(0),
              () -> chassisVelocities[0],
              () -> 1000L,
              List.of(poseObs::addAll),
              List.of());

      Field field = new Field(null, null, null, null, 16.0, 8.0, null, List.of());
      camera.setFieldLayoutSupplier(() -> field);

      doAnswer(
              inv -> {
                var in = (LimelightIOInputsAutoLogged) inv.getArgument(0);
                in.mt1PoseEstimate = inputs.mt1PoseEstimate;
                in.mt2PoseEstimate = inputs.mt2PoseEstimate;
                in.rawFiducials = inputs.rawFiducials;
                return null;
              })
          .when(io)
          .updateInputs(any(CameraIO.LimelightIOInputs.class));

      // 1) MT2 rejected when yaw rate > 720 deg/s (13 rad/s)
      chassisVelocities[0] = new ChassisVelocities(0, 0, 13.0);
      timestamp[0] = 1.0;
      CameraIO.RawFiducial goodFid = new CameraIO.RawFiducial(1, 0, 0, 1, 2, 2, 0.05);
      inputs.mt2PoseEstimate =
          new CameraIO.PoseEstimate(
              new Pose2d(5, 5, new Rotation2d()),
              timestamp[0],
              5,
              1,
              1,
              2,
              0.5,
              new CameraIO.RawFiducial[] {goodFid},
              true);
      inputs.rawFiducials = new CameraIO.RawFiducial[] {goodFid};

      poseObs.clear();
      camera.periodic();
      assertTrue(poseObs.isEmpty()); // Rejected due to high yaw rate!

      // 2) MT1 single-tag rejected when ambiguity > 0.4
      chassisVelocities[0] = new ChassisVelocities(0, 0, 0);
      timestamp[0] = 2.0;
      inputs.mt2PoseEstimate = new CameraIO.PoseEstimate(); // MT2 empty
      CameraIO.RawFiducial ambiguousFid =
          new CameraIO.RawFiducial(1, 0, 0, 1, 2, 2, 0.6); // 0.6 > 0.4
      inputs.mt1PoseEstimate =
          new CameraIO.PoseEstimate(
              new Pose2d(5, 5, new Rotation2d()),
              timestamp[0],
              5,
              1,
              1,
              2,
              0.5,
              new CameraIO.RawFiducial[] {ambiguousFid},
              false);
      inputs.rawFiducials = new CameraIO.RawFiducial[] {ambiguousFid};

      poseObs.clear();
      camera.periodic();
      assertTrue(poseObs.isEmpty()); // Rejected due to high ambiguity!

      // 3) Out of bounds rejection
      timestamp[0] = 3.0;
      inputs.mt1PoseEstimate =
          new CameraIO.PoseEstimate(
              new Pose2d(20.0, 5, new Rotation2d()),
              timestamp[0],
              5,
              1,
              1,
              2,
              0.5,
              new CameraIO.RawFiducial[] {goodFid},
              false);
      poseObs.clear();
      camera.periodic();
      assertTrue(poseObs.isEmpty()); // Rejected because X=20 > 16.5

      // 4) Max distance rejection (> 10m)
      timestamp[0] = 4.0;
      inputs.mt1PoseEstimate =
          new CameraIO.PoseEstimate(
              new Pose2d(5, 5, new Rotation2d()),
              timestamp[0],
              5,
              1,
              1,
              12.0,
              0.5,
              new CameraIO.RawFiducial[] {goodFid},
              false);
      poseObs.clear();
      camera.periodic();
      assertTrue(poseObs.isEmpty()); // Rejected because avgTagDist=12.0 > 10.0
    }
  }
}
