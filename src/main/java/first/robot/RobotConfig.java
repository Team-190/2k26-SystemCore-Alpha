package first.robot;

public final class RobotConfig {
  public static final RobotType ROBOT = RobotType.V3_Horse;

  public enum RobotType {
    V3_Horse,
    V3_Horse_SIM;

    RobotType() {}
  }
}
