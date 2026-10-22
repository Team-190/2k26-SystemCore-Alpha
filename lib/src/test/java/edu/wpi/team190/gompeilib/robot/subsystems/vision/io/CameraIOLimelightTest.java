package edu.wpi.team190.gompeilib.robot.subsystems.vision.io;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.limelightvision.FiducialTarget;
import com.limelightvision.IMUMode;
import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import com.limelightvision.PoseEstimateType;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.system.Timer;

public class CameraIOLimelightTest {

  @BeforeEach
  public void setUp() {
    CameraIOLimelight.clearInstances();
  }

  @Test
  public void testConfigConstructors() {
    VisionConstants.StaticLimelightConfig staticConfig =
        VisionConstants.StaticLimelightConfig.builder()
            .key("static_ll")
            .robotToCameraTransform(new Transform3d())
            .build();

    CameraIOLimelight staticIO = new CameraIOLimelight(staticConfig);
    assertEquals("limelight-static_ll", staticIO.getName());

    VisionConstants.MovingLimelightConfig movingConfig =
        VisionConstants.MovingLimelightConfig.builder()
            .key("moving_ll")
            .robotToRotationAxisTransform(new Transform3d())
            .rotationAxisToLensTransform(new Transform3d())
            .build();

    CameraIOLimelight movingIO = new CameraIOLimelight(movingConfig);
    assertEquals("limelight-moving_ll", movingIO.getName());
  }

  @Test
  public void testUpdateInputsWithResults() {
    Limelight mockLimelight = mock(Limelight.class);
    CameraIOLimelight io = new CameraIOLimelight("test", mockLimelight);

    LimelightResults results = new LimelightResults();
    when(mockLimelight.getLatestResults()).thenReturn(results);

    com.limelightvision.PoseEstimate mockMt1 = new com.limelightvision.PoseEstimate();
    mockMt1.pose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0));
    mockMt1.timestampSeconds = 12.34;
    mockMt1.latencyMillis = 15.0;
    mockMt1.reportedTagCount = 1;
    mockMt1.tagSpanMeters = 1.0;
    mockMt1.avgTagDistanceMeters = 2.0;
    mockMt1.avgTagAreaPercent = 0.5;
    FiducialTarget target1 = new FiducialTarget();
    target1.fiducialId = 5;
    target1.txDegreesNoCrosshair = 1.0;
    target1.tyDegreesNoCrosshair = 2.0;
    target1.targetAreaPercent = 0.5;
    target1.ambiguity = 0.1;
    target1.targetPoseCameraSpace = new double[] {0, 0, 2.0, 0, 0, 0};
    target1.targetPoseRobotSpace = new double[] {0, 0, 2.0, 0, 0, 0};
    mockMt1.rawFiducials = new FiducialTarget[] {target1};

    com.limelightvision.PoseEstimate mockMt2 = new com.limelightvision.PoseEstimate();
    mockMt2.pose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(45));
    mockMt2.timestampSeconds = 12.34;
    mockMt2.reportedTagCount = 1;
    mockMt2.type = PoseEstimateType.MT2_WPIBLUE;
    mockMt2.rawFiducials = new FiducialTarget[] {};

    when(mockLimelight.getPoseEstimate(results, PoseEstimateType.MT1_WPIBLUE)).thenReturn(mockMt1);
    when(mockLimelight.getPoseEstimate(results, PoseEstimateType.MT2_WPIBLUE)).thenReturn(mockMt2);

    CameraIO.LimelightIOInputs inputs = new CameraIO.LimelightIOInputs();
    io.updateInputs(inputs);

    assertEquals(1.0, inputs.mt1PoseEstimate.pose().getX());
    assertEquals(2.0, inputs.mt1PoseEstimate.pose().getY());
    assertEquals(3.0, inputs.mt2PoseEstimate.pose().getX());
    assertEquals(4.0, inputs.mt2PoseEstimate.pose().getY());
    assertTrue(inputs.mt2PoseEstimate.isMegaTag2());
    assertFalse(inputs.mt1PoseEstimate.isMegaTag2());
    assertEquals(1, inputs.rawFiducials.length);
    assertEquals(5, inputs.rawFiducials[0].id());

    // Test fallback to MT2 fiducials when MT1 has no fiducials
    mockMt1.rawFiducials = new FiducialTarget[0];
    FiducialTarget target2 = new FiducialTarget();
    target2.fiducialId = 7;
    mockMt2.rawFiducials = new FiducialTarget[] {target2};

    CameraIO.LimelightIOInputs fallbackInputs = new CameraIO.LimelightIOInputs();
    io.updateInputs(fallbackInputs);
    assertEquals(1, fallbackInputs.rawFiducials.length);
    assertEquals(7, fallbackInputs.rawFiducials[0].id());
  }

  @Test
  public void testUpdateInputsNullHandling() {
    Limelight mockLimelight = mock(Limelight.class);
    CameraIOLimelight io = new CameraIOLimelight("test", mockLimelight);
    when(mockLimelight.getLatestResults()).thenReturn(null);

    CameraIO.LimelightIOInputs inputs = new CameraIO.LimelightIOInputs();
    io.updateInputs(inputs);
    assertEquals(0, inputs.rawFiducials.length);

    CameraIOLimelight nullLimelightIO = new CameraIOLimelight("test", null);
    nullLimelightIO.updateInputs(inputs);
    assertEquals(0, inputs.rawFiducials.length);
  }

  @Test
  public void testHardwareControlDelegatesSingleInstance() {
    Limelight mockLimelight = mock(Limelight.class);
    CameraIOLimelight io = new CameraIOLimelight("test", mockLimelight);

    Pose3d pose = new Pose3d();
    io.setCameraPose(pose);
    verify(mockLimelight).setCameraPose_RobotSpaceOverride(pose, false);

    // Null pose should not call through
    io.setCameraPose(null);
    verify(mockLimelight, times(1))
        .setCameraPose_RobotSpaceOverride(any(Pose3d.class), anyBoolean());

    // Single instance flushes on robot orientation set
    io.setRobotOrientation(15.0, 2.5);
    verify(mockLimelight).setRobotOrientation(15.0, 2.5, 0.0, 0.0, 0.0, 0.0, true);

    io.setRobotOrientation(
        org.wpilib.units.Units.Degrees.of(15.0), org.wpilib.units.Units.DegreesPerSecond.of(2.5));
    verify(mockLimelight, times(2)).setRobotOrientation(15.0, 2.5, 0.0, 0.0, 0.0, 0.0, true);

    io.setThrottle(190);
    verify(mockLimelight).setThrottle(190);

    io.setIMUMode(0);
    verify(mockLimelight).setIMUMode(IMUMode.EXTERNAL);

    io.setIMUMode(1);
    verify(mockLimelight).setIMUMode(IMUMode.EXTERNAL_SEED_INTERNAL);

    io.setIMUMode(IMUMode.INTERNAL_EXTERNAL_ASSIST);
    verify(mockLimelight).setIMUMode(IMUMode.INTERNAL_EXTERNAL_ASSIST);

    io.setIMUMode((IMUMode) null);

    io.setIMUAssistAlpha(0.005);
    verify(mockLimelight).setIMUAssistAlpha(0.005);

    io.setRewindEnabled(true);
    verify(mockLimelight).setRewindEnabled(true);

    io.triggerRewindCapture(165.0);
    verify(mockLimelight).triggerRewindCapture(165.0);
  }

  @Test
  public void testMultipleInstancesFlushCoordination() {
    Limelight mockLimelight1 = mock(Limelight.class);
    Limelight mockLimelight2 = mock(Limelight.class);
    CameraIOLimelight io1 = new CameraIOLimelight("cam1", mockLimelight1);
    CameraIOLimelight io2 = new CameraIOLimelight("cam2", mockLimelight2);

    try (MockedStatic<Limelight> mockLimelightStatic = mockStatic(Limelight.class);
        MockedStatic<Timer> mockTimer = mockStatic(Timer.class)) {
      mockTimer.when(Timer::getTimestamp).thenReturn(1.0);

      // Camera 1 updates: multiple instances present, so flush is false!
      io1.setRobotOrientation(10.0, 1.0);
      verify(mockLimelight1).setRobotOrientation(10.0, 1.0, 0.0, 0.0, 0.0, 0.0, false);
      mockLimelightStatic.verify(Limelight::flushNT, never());

      // Camera 2 updates: second of two instances, so flushNT is triggered once!
      io2.setRobotOrientation(20.0, 2.0);
      verify(mockLimelight2).setRobotOrientation(20.0, 2.0, 0.0, 0.0, 0.0, 0.0, false);
      mockLimelightStatic.verify(Limelight::flushNT, times(1));

      // Explicit flush now does nothing extra because needsFlush was cleared
      CameraIOLimelight.flushNT();
      mockLimelightStatic.verify(Limelight::flushNT, times(1));

      // Timestamp advance: new cycle where camera 1 updates camera pose
      mockTimer.when(Timer::getTimestamp).thenReturn(2.0);
      io1.setCameraPose(new Pose3d());
      io1.flush();
      mockLimelightStatic.verify(Limelight::flushNT, times(2));
    }
  }

  @Test
  public void testSetSharedRobotOrientation() {
    Limelight mockLimelight = mock(Limelight.class);
    CameraIOLimelight io = new CameraIOLimelight("test", mockLimelight);

    try (MockedStatic<Limelight> mockLimelightStatic = mockStatic(Limelight.class);
        MockedStatic<Timer> mockTimer = mockStatic(Timer.class)) {
      mockTimer.when(Timer::getTimestamp).thenReturn(1.0);

      // First call in cycle 1.0 -> published
      io.setSharedRobotOrientation(12.0, 3.4);
      mockLimelightStatic.verify(
          () -> Limelight.setSharedRobotOrientation(12.0, 3.4, 0.0, 0.0, 0.0, 0.0), times(1));

      // Second call at same timestamp 1.0 -> deduplicated, not published again
      io.setSharedRobotOrientation(15.0, 5.0);
      mockLimelightStatic.verify(
          () ->
              Limelight.setSharedRobotOrientation(
                  anyDouble(), anyDouble(), anyDouble(), anyDouble(), anyDouble(), anyDouble()),
          times(1));

      // Advance timestamp to 2.0 -> new cycle publishes
      mockTimer.when(Timer::getTimestamp).thenReturn(2.0);
      io.setSharedRobotOrientation(15.0, 5.0);
      mockLimelightStatic.verify(
          () -> Limelight.setSharedRobotOrientation(15.0, 5.0, 0.0, 0.0, 0.0, 0.0), times(1));

      // Advance timestamp to 3.0 -> measure overload publishes
      mockTimer.when(Timer::getTimestamp).thenReturn(3.0);
      io.setSharedRobotOrientation(
          org.wpilib.units.Units.Degrees.of(15.0), org.wpilib.units.Units.DegreesPerSecond.of(5.0));
      mockLimelightStatic.verify(
          () -> Limelight.setSharedRobotOrientation(15.0, 5.0, 0.0, 0.0, 0.0, 0.0), times(2));
    }
  }

  @Test
  public void testNullLimelightDelegates() {
    CameraIOLimelight io = new CameraIOLimelight("test", null);
    assertDoesNotThrow(
        () -> {
          io.setCameraPose(new Pose3d());
          io.setRobotOrientation(0.0, 0.0);
          io.setSharedRobotOrientation(0.0, 0.0);
          io.setRobotOrientation(
              org.wpilib.units.Units.Degrees.of(0.0),
              org.wpilib.units.Units.DegreesPerSecond.of(0.0));
          io.setSharedRobotOrientation(
              org.wpilib.units.Units.Degrees.of(0.0),
              org.wpilib.units.Units.DegreesPerSecond.of(0.0));
          io.setThrottle(0);
          io.setIMUMode(0);
          io.setIMUMode(IMUMode.EXTERNAL_SEED_INTERNAL);
          io.setIMUAssistAlpha(0.001);
          io.setRewindEnabled(false);
          io.triggerRewindCapture(0);
          io.flush();
        });
  }

  @Test
  public void testDataRecords() {
    CameraIO.PoseEstimate defaultPose = new CameraIO.PoseEstimate();
    assertEquals(0.0, defaultPose.timestampSeconds());
    assertEquals(0, defaultPose.rawFiducials().length);

    CameraIO.PoseEstimate nullWrapper =
        new CameraIO.PoseEstimate((com.limelightvision.PoseEstimate) null);
    assertEquals(0.0, nullWrapper.timestampSeconds());
    assertEquals(Pose2d.ZERO, nullWrapper.pose());

    CameraIO.RawFiducial defaultFid = new CameraIO.RawFiducial();
    assertEquals(0, defaultFid.id());

    CameraIO.RawFiducial nullFid = new CameraIO.RawFiducial((FiducialTarget) null);
    assertEquals(0, nullFid.id());

    CameraIO.RawDetection defaultDet = new CameraIO.RawDetection();
    assertEquals(0, defaultDet.classId());
  }
}
