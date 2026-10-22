package edu.wpi.team190.gompeilib.robot.subsystems.vision.io;

import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.io.limelight.LimelightHelpers;
import lombok.Getter;

public class CameraIOLimelight implements CameraIO {
  @Getter private final String name;

  public CameraIOLimelight(VisionConstants.StaticLimelightConfig config) {
    this.name = "limelight-" + config.key();
  }

  public CameraIOLimelight(VisionConstants.MovingLimelightConfig config) {
    this.name = "limelight-" + config.key();
  }

  @Override
  public void updateInputs(LimelightIOInputs inputs) {
    inputs.mt1PoseEstimate = new PoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue(name));
    inputs.mt2PoseEstimate =
        new PoseEstimate(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name));
    inputs.rawFiducials =
        inputs.mt1PoseEstimate.rawFiducials().length > 0
            ? inputs.mt1PoseEstimate.rawFiducials()
            : inputs.mt2PoseEstimate.rawFiducials();
  }
}
