package edu.wpi.team190.gompeilib.robot.subsystems.vision.io;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.limelight.LimelightHelpers;
import org.junit.jupiter.api.Test;
import org.mockito.MockedStatic;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;

public class CameraIOLimelightTest {

  @Test
  public void testCameraIOLimelightStaticConfig() {
    VisionConstants.StaticLimelightConfig config =
        VisionConstants.StaticLimelightConfig.builder()
            .key("static_ll")
            .robotToCameraTransform(new Transform3d())
            .build();

    CameraIOLimelight io = new CameraIOLimelight(config);
    assertEquals("limelight-static_ll", io.getName());

    try (MockedStatic<LimelightHelpers> mockHelpers = mockStatic(LimelightHelpers.class)) {
      LimelightHelpers.PoseEstimate mockEst1 =
          new LimelightHelpers.PoseEstimate(
              new Pose2d(1.0, 2.0, new org.wpilib.math.geometry.Rotation2d()),
              1.0,
              2.0,
              1,
              0.0,
              0.0,
              0.0,
              new LimelightHelpers.RawFiducial[] {
                new LimelightHelpers.RawFiducial(5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
              },
              false);

      LimelightHelpers.PoseEstimate mockEst2 =
          new LimelightHelpers.PoseEstimate(
              new Pose2d(3.0, 4.0, new org.wpilib.math.geometry.Rotation2d()),
              3.0,
              4.0,
              2,
              0.0,
              0.0,
              0.0,
              new LimelightHelpers.RawFiducial[] {
                new LimelightHelpers.RawFiducial(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
              },
              true);

      LimelightHelpers.RawFiducial[] mockFids =
          new LimelightHelpers.RawFiducial[] {
            new LimelightHelpers.RawFiducial(7, 1.0, 2.0, 3.0, 4.0, 5.0, 0.6)
          };

      mockHelpers
          .when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-static_ll"))
          .thenReturn(mockEst1);
      mockHelpers
          .when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-static_ll"))
          .thenReturn(mockEst2);
      mockHelpers
          .when(() -> LimelightHelpers.getRawFiducials("limelight-static_ll"))
          .thenReturn(mockFids);

      CameraIO.LimelightIOInputs inputs = new CameraIO.LimelightIOInputs();
      io.updateInputs(inputs);

      assertEquals(1.0, inputs.mt1PoseEstimate.pose().getX());
      assertEquals(3.0, inputs.mt2PoseEstimate.pose().getX());
      assertEquals(1, inputs.rawFiducials.length);
      assertEquals(5, inputs.rawFiducials[0].id());

      // Test fallback to MT2 fiducials when MT1 has no fiducials
      LimelightHelpers.PoseEstimate emptyMT1 =
          new LimelightHelpers.PoseEstimate(
              new Pose2d(), 0.0, 0.0, 0, 0.0, 0.0, 0.0, new LimelightHelpers.RawFiducial[0], false);
      mockHelpers
          .when(() -> LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-static_ll"))
          .thenReturn(emptyMT1);
      CameraIO.LimelightIOInputs fallbackInputs = new CameraIO.LimelightIOInputs();
      io.updateInputs(fallbackInputs);
      assertEquals(1, fallbackInputs.rawFiducials.length);
      assertEquals(6, fallbackInputs.rawFiducials[0].id());
    }
  }

  @Test
  public void testCameraIOLimelightMovingConfig() {
    VisionConstants.MovingLimelightConfig config =
        VisionConstants.MovingLimelightConfig.builder()
            .key("moving_ll")
            .robotToRotationAxisTransform(new Transform3d())
            .rotationAxisToLensTransform(new Transform3d())
            .build();

    CameraIOLimelight io = new CameraIOLimelight(config);
    assertEquals("limelight-moving_ll", io.getName());
  }
}
