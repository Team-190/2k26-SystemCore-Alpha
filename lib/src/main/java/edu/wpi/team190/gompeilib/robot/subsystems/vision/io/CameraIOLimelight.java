package edu.wpi.team190.gompeilib.robot.subsystems.vision.io;

import com.limelightvision.IMUMode;
import com.limelightvision.Limelight;
import com.limelightvision.LimelightResults;
import com.limelightvision.PoseEstimateType;
import edu.wpi.team190.gompeilib.robot.subsystems.vision.VisionConstants;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.WeakHashMap;
import lombok.Getter;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.system.Timer;

public class CameraIOLimelight implements CameraIO {
  private static final Set<CameraIOLimelight> instances =
      Collections.synchronizedSet(Collections.newSetFromMap(new WeakHashMap<>()));
  private static final Set<CameraIOLimelight> updatedThisCycle = new HashSet<>();
  private static double lastCycleTimestamp = -1.0;
  private static double lastSharedOrientationTimestamp = -1.0;
  private static boolean needsFlush = false;

  @Getter private final String name;
  private final Limelight limelight;

  public CameraIOLimelight(VisionConstants.StaticLimelightConfig config) {
    this(config.key(), new Limelight("limelight-" + config.key()));
  }

  public CameraIOLimelight(VisionConstants.MovingLimelightConfig config) {
    this(config.key(), new Limelight("limelight-" + config.key()));
  }

  public CameraIOLimelight(String key, Limelight limelight) {
    this.name = "limelight-" + key;
    this.limelight = limelight;
    instances.add(this);
  }

  public static void clearInstances() {
    instances.clear();
    updatedThisCycle.clear();
    needsFlush = false;
    lastCycleTimestamp = -1.0;
    lastSharedOrientationTimestamp = -1.0;
  }

  public static void flushNT() {
    if (needsFlush) {
      Limelight.flushNT();
      needsFlush = false;
    }
  }

  @Override
  public void flush() {
    flushNT();
  }

  @Override
  public void updateInputs(LimelightIOInputs inputs) {
    LimelightResults results = limelight != null ? limelight.getLatestResults() : null;
    if (results != null) {
      com.limelightvision.PoseEstimate mt1 =
          limelight.getPoseEstimate(results, PoseEstimateType.MT1_WPIBLUE);
      com.limelightvision.PoseEstimate mt2 =
          limelight.getPoseEstimate(results, PoseEstimateType.MT2_WPIBLUE);

      inputs.mt1PoseEstimate = new PoseEstimate(mt1);
      inputs.mt2PoseEstimate = new PoseEstimate(mt2);
      inputs.rawFiducials =
          inputs.mt1PoseEstimate.rawFiducials().length > 0
              ? inputs.mt1PoseEstimate.rawFiducials()
              : inputs.mt2PoseEstimate.rawFiducials();
    }
  }

  @Override
  public void setCameraPose(Pose3d pose) {
    if (limelight != null && pose != null) {
      limelight.setCameraPose_RobotSpaceOverride(pose, false);
      needsFlush = true;
    }
  }

  @Override
  public void setRobotOrientation(
      org.wpilib.units.measure.Angle yaw, org.wpilib.units.measure.AngularVelocity yawRate) {
    setRobotOrientation(
        yaw.in(org.wpilib.units.Units.Degrees),
        yawRate.in(org.wpilib.units.Units.DegreesPerSecond));
  }

  @Override
  public void setRobotOrientation(double yawDegrees, double yawRateDegreesPerSec) {
    if (limelight != null) {
      boolean multiple = instances.size() > 1;
      limelight.setRobotOrientation(
          yawDegrees, yawRateDegreesPerSec, 0.0, 0.0, 0.0, 0.0, !multiple);
      if (multiple) {
        needsFlush = true;
        double now = Timer.getTimestamp();
        if (now != lastCycleTimestamp) {
          lastCycleTimestamp = now;
          updatedThisCycle.clear();
        }
        updatedThisCycle.add(this);
        if (updatedThisCycle.size() >= instances.size()) {
          flushNT();
          updatedThisCycle.clear();
        }
      } else {
        needsFlush = false;
      }
    }
  }

  @Override
  public void setSharedRobotOrientation(
      org.wpilib.units.measure.Angle yaw, org.wpilib.units.measure.AngularVelocity yawRate) {
    setSharedRobotOrientation(
        yaw.in(org.wpilib.units.Units.Degrees),
        yawRate.in(org.wpilib.units.Units.DegreesPerSecond));
  }

  @Override
  public void setSharedRobotOrientation(double yawDegrees, double yawRateDegreesPerSec) {
    if (limelight != null) {
      double now = Timer.getTimestamp();
      if (now != lastSharedOrientationTimestamp) {
        lastSharedOrientationTimestamp = now;
        Limelight.setSharedRobotOrientation(yawDegrees, yawRateDegreesPerSec, 0.0, 0.0, 0.0, 0.0);
      }
    }
  }

  @Override
  public void setThrottle(int throttle) {
    if (limelight != null) {
      limelight.setThrottle(throttle);
    }
  }

  @Override
  public void setIMUMode(IMUMode mode) {
    if (limelight != null && mode != null) {
      limelight.setIMUMode(mode);
    }
  }

  @Override
  public void setIMUMode(int mode) {
    CameraIO.super.setIMUMode(mode);
  }

  @Override
  public void setIMUAssistAlpha(double alpha) {
    if (limelight != null) {
      limelight.setIMUAssistAlpha(alpha);
    }
  }

  @Override
  public void setRewindEnabled(boolean enabled) {
    if (limelight != null) {
      limelight.setRewindEnabled(enabled);
    }
  }

  @Override
  public void triggerRewindCapture(double durationSeconds) {
    if (limelight != null) {
      limelight.triggerRewindCapture(durationSeconds);
    }
  }
}
