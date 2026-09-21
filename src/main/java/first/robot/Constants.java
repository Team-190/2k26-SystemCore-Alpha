package first.robot;

import org.wpilib.framework.RobotBase;

import edu.wpi.team190.gompeilib.core.robot.RobotMode;

public final class Constants {
  public static final boolean TUNING_MODE = false;
  public static final double LOOP_PERIOD_SECONDS = 0.02;

  public static RobotMode getMode() {
    switch (RobotConfig.ROBOT) {
      case V3_Horse_CV2_TR:
        return RobotBase.isReal() ? RobotMode.REAL : RobotMode.REPLAY;

      case V3_Horse_CV2_TR_SIM:
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
