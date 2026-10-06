package first.robot.v3_Horse_CV2_TR;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

public class V3_Horse_CV2_TRMechanism3D {

  public static Pose3d[] getPoses(Distance intakePosition, Angle hoodAngle) {
    return new Pose3d[] {Pose3d.ZERO, Pose3d.ZERO, Pose3d.ZERO};
  }
}
