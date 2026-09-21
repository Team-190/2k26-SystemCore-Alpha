package first.robot;

public final class RobotConfig {
  public static final RobotType ROBOT = RobotType.V3_Horse_CV2_TR;

  public enum RobotType {
    V3_Horse_CV2_TR,
    V3_Horse_CV2_TR_SIM;

    RobotType() {}
    
  }
}
