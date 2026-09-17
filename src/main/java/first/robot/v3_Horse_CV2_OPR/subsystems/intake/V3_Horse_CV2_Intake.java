package first.robot.v3_Horse_CV2_OPR.subsystems.intake;

import static org.wpilib.units.Units.*;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.util.function.BooleanConsumer;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.subsystems.extension.Extension;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import 

import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;


public class V3_Horse_CV2_Intake extends SubsystemBase{


    private final GenericRoller intakeRoller;

    private final GenericRoller kickerRoller;

    public record IntakeStateSetter(
        BooleanConsumer stowed,
        BooleanConsumer collecting,
        BooleanConsumer spitting,
        BooleanConsumer slowCollecting){
            public IntakeStateSetter(){
                this(b -> {}, b -> {}, b -> {}, b -> {});
            }
        }


    public Intake(GenericRollerIO rollerIO, 
        ExtensionIO extensionIO){
        setName("Intake");
    };

    intakeState = IntakeState.STOW;




    intakeRoller =
        new GenericRoller(
            rollerIO, this, IntakeConstants.INTAKE_ROLLER_CONSTANTS, "", normalVoltageSetpoint);
    
    kickerRoller =
        new GenericRoller(
            rollerIO, this, IntakeConstants.KICKER_ROLLER_CONSTANTS, "", normalVoltageSetpoint);
    

}