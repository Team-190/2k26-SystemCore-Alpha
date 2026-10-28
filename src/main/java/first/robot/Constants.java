package first.robot;

import edu.wpi.team190.gompeilib.robot.RobotMode;
import org.wpilib.framework.RobotBase;

public final class Constants {
  public static final boolean TUNING_MODE = true;
  public static final double LOOP_PERIOD_SECONDS = 0.005;

  public static RobotMode getMode() {
    switch (RobotConfig.ROBOT) {
      case V3_Horse:
        return RobotBase.isReal() ? RobotMode.REAL : RobotMode.REPLAY;

      case V3_Horse_SIM:
        return RobotMode.SIM;

      default:
        return RobotMode.REAL;
    }
  }

  public static void main(String... args) {
    if (getMode().equals(RobotMode.SIM)) {
      System.err.println("Cannot deploy, invalid mode selected: " + RobotConfig.ROBOT);
      System.exit(1);
    }
  }
}
