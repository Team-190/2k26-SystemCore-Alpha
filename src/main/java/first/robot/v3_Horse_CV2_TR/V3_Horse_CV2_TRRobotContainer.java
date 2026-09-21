package first.robot.v3_Horse_CV2_TR;

import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.core.robot.RobotMode;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOSim;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIOTalonFX;
import first.robot.Constants;
import first.robot.RobotConfig;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloorConstants;

public class V3_Horse_CV2_TRRobotContainer implements RobotContainer {
  private V3_Horse_CV2_TR_RollerFloor rollerFloor;

  public V3_Horse_CV2_TRRobotContainer() {
    if (Constants.getMode() != RobotMode.REPLAY) {
      switch (RobotConfig.ROBOT) {
        case V3_Horse_CV2_TR:
          rollerFloor =
              new V3_Horse_CV2_TR_RollerFloor(
                  new GenericRollerIOTalonFX(
                      V3_Horse_CV2_TR_RollerFloorConstants.ROLLER_FLOOR_CONSTANTS));
          break;
        case V3_Horse_CV2_TR_SIM:
          rollerFloor =
              new V3_Horse_CV2_TR_RollerFloor(
                  new GenericRollerIOSim(
                      V3_Horse_CV2_TR_RollerFloorConstants.ROLLER_FLOOR_CONSTANTS));
          break;
        default:
          break;
      }
    }
  }
}
