package first.robot;

public final class RobotConfig {
  public static final RobotType ROBOT = RobotType.V3_Horse_CV2_TR;

  public enum RobotType {
    V3_Horse_CV2_TR,
    V3_Horse_CV2_TR_SIM,
    V3_Horse_CV3_TR,
    V3_Horse_CV3_TR_SIM,
    V3_Horse_CV2_OPR,
    V3_Horse_CV2_OPR_SIM,
    V3_Horse_CV3_OPR,
    V3_Horse_CV3_OPR_SIM;

    RobotType() {}
  }
}
