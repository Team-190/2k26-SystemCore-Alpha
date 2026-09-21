package first.robot.v3_Horse_CV2_OPR.subsystems.intake;

import static org.wpilib.units.Units.*;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.VoltageUnit;
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

import first.robot.v3_Horse_CV2_OPR.subsystems.intake.V3_Horse_CV2_IntakeConstants.IntakeState;
import first.robot.v3_Horse_CV2_OPR.subsystems.intake.V3_Horse_CV2_IntakeConstants;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;


public class V3_Horse_CV2_Intake extends SubsystemBase{


    @Getter private IntakeState intakeState;

    private final GenericRoller intakeRoller;

    public static final Extension extension;

    private final Setpoint<VoltageUnit> voltageSetpoint;

    private final Setpoint<DistanceUnit> positionSetpoint;

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


    public void Intake(GenericRollerIO intakeRollerIO, GenericRollerIO kickerRollerIO,
        ExtensionIO extensionIO){
        setName("Intake");
    

    intakeState = IntakeState.STOW;


    voltageSetpoint = new Setpoint<>(Volts.of(0.0), V3_Horse_CV2_IntakeConstants.INTAKE_ROLLER_CONSTANTS.voltageOffsetStep, Volts.of(-12.0), Volts.of(12.0));

    positionSetpoint = new Setpoint<>(Meters.of(0.0), V3_Horse_CV2_IntakeConstants.EXTENSION_CONSTANTS.heightOffsetStep, Meters.of(0.01), Meters.of(0.01));


    intakeRoller =
        new GenericRoller(
            intakeRollerIO, this, V3_Horse_CV2_IntakeConstants.INTAKE_ROLLER_CONSTANTS, "", voltageSetpoint);
    
    kickerRoller =
        new GenericRoller(
            kickerRollerIO, this, V3_Horse_CV2_IntakeConstants.KICKER_ROLLER_CONSTANTS, "", voltageSetpoint);
        
    extension =
        new Extension(V3_Horse_CV2_IntakeConstants.EXTENSION_CONSTANTS, this, 3, extensionIO, positionSetpoint, VoltageSetpoint);
        
    
    }
    @Trace
    @Override
    public void periodic() {
        intakeRoller.periodic();
        kickerRoller.periodic();
        extension.periodic();
    Logger.recordOutput("Intake/State", intakeState.toString());
    }


    public Command setIntakeVoltage(double voltage){
        return Commands.runOnce(() -> intakeRoller.setVoltageGoal(Volts.of(voltage)));
    }

    public Command setKickerVoltage(double voltage){
        return Commands.runOnce(() -> kickerRoller.setVoltageGoal(Volts.of(voltage)));
    }

  public Command deploy() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              intakeState = IntakeState.INTAKE;
              extension.setPositionGoal(V3_Horse_CV2_IntakeConstants.INTAKE_STATES.get(IntakeState.INTAKE));
            }));
  }
}

