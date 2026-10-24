package first.robot.v3_Horse_CV2_TR;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

public class V3_Horse_CV2_TRMechanism3D {
  private static final Pose3d baseIntakePose = new Pose3d(0.295732, 0, 0.214122, Rotation3d.ZERO);
  private static final Rotation3d baseIntakeRotationFromX =
      new Rotation3d(Degrees.zero(), Degrees.of(8.352861), Degrees.zero());

  private static boolean hasKickerDeployed = false;
  private static Distance intakeKickerDeployDistance = Meters.of(0.2);
  private static final Pose3d baseKickerPose = new Pose3d(0.230747, 0, 0.183872, Rotation3d.ZERO);
  private static final Pose3d deployedKickerPose =
      baseKickerPose.rotateAround(
          baseKickerPose.getTranslation(),
          new Rotation3d(Degrees.zero(), Degrees.of(115.894967), Degrees.zero()));

  private static final Pose3d baseHoodPose = new Pose3d(-0.263527, 0, 0.476161, Rotation3d.ZERO);

  public static Pose3d[] getPoses(Distance intakePosition, Angle hoodAngle) {
    Translation3d currentIntakeTranslation =
        new Translation3d(intakePosition, Meters.zero(), Meters.zero())
            .rotateBy(baseIntakeRotationFromX);
    Pose3d currentIntakePose =
        baseIntakePose.transformBy(new Transform3d(currentIntakeTranslation, Rotation3d.ZERO));

    hasKickerDeployed = hasKickerDeployed || intakePosition.gte(intakeKickerDeployDistance);

    Pose3d currentHoodPose =
        baseHoodPose.rotateAround(
            baseHoodPose.getTranslation(),
            new Rotation3d(Degrees.of(0), hoodAngle.unaryMinus(), Degrees.of(0)));

    return new Pose3d[] {
      currentIntakePose, hasKickerDeployed ? deployedKickerPose : baseKickerPose, currentHoodPose
    };
  }
}
