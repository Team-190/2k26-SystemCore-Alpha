package first.robot.v3_Horse_CV2_TR.commands;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloorConstants.RollerFloorState;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooter;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants.ShooterGoal;

public class V3_Horse_CV2_TRCompositeCommands {

    public static Command shootCommand(V3_Horse_CV2_TR_RollerFloor rollerFloor, V3_Horse_CV2_TRShooter shooter){
        return Commands.sequence(shooter.setGoal(ShooterGoal.SCORE), shooter.waitUntilAtGoal(), rollerFloor.setState(RollerFloorState.RUN));

    }

    public static Command stopShooter(V3_Horse_CV2_TR_RollerFloor rollerFloor, V3_Horse_CV2_TRShooter shooter){
        return Commands.parallel(shooter.setGoal(ShooterGoal.STOP), rollerFloor.setState(RollerFloorState.STOP));
    }

    public static Command shootWithAgitateCommand(V3_Horse_CV2_TR_RollerFloor rollerFloor, V3_Horse_CV2_TRShooter shooter, V3_Horse_CV2_TR_Intake intake){
        return Commands.parallel(Commands.sequence(shooter.setGoal(ShooterGoal.SCORE), 
        shooter.waitUntilAtGoal(), rollerFloor.setState(RollerFloorState.RUN)), 
        intake.setState(intakeState.AGITATE));

    }

}


