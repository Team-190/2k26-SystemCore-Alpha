package first.robot.v3_Horse_CV2_OPR.subsystems.intake;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Units;
import org.wpilib.units.Unit;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.subsystems.extension.Extension;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import java.util.Map;


public class V3_Horse_CV2_IntakeConstants {
    public static final double INTAKE_VOLTAGE;
    public static final double EXTAKE_VOLTAGE;

    public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS;
    public static final int MOTOR_CAN_ID;
    public static final double GEAR_RATIO;

    public static final double SUPPLY_CURRENT_LIMIT;
    public static final double STATOR_CURRENT_LIMIT;

    public static final double MOMENT_OF_INERTIA;
    public static final DCMotor MOTOR_CONFIG;

    public static final Extension EXTENSION;

    public static final Translation2d MAX_EXTENSION;
    public static final Translation2d MIN_EXTENSION;

    public static final Translation2d EXTENSION_INCREMENT;



    public static final Map<IntakeState, Setpoint<DistanceUnit>> INTAKE_STATES;

    public static final Gains GAINS;


    static{
    INTAKE_VOLTAGE = 12.0;
    EXTAKE_VOLTAGE = -12.0;

    INTAKE_ROLLER_CONSTANTS = 
        GenericRollerConstants.builder()
        .withLeaderCANID(40) //not sure if these are taken or not
        .withCurrentLimits(
            CurrentLimits.builder()
                .withSupplyCurrentLimit(Amps.of(40.0))
                .withStatorCurrentLimit(Amps.of(40.0))
                .build())
        .withNeutralMode(NeutralModeValue.Coast)
        .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
        .withRollerMotorGearRatio((1.0/2.0)) //please god someone change this to be realistic
        .withInvertedValue(InvertedValue.CounterClockwise_Positive)
        .withOpposedFollowerCANID(41)
        .withMomentOffInertia(Units.KilogramSquareMeters.of(0.0004))
        .withVOltageOffsetStep(Volts.of(1))
        .withCanBus(CANBus.roboRIO())
        .withEnableFOC(false)
        .build();

    INTAKE_STATES = 
        Map.of(
            IntakeState.STOW,
            new Setpoint<>(
            ),


        IntakeState.INTAKE,
            new Setpoint<>(),
        IntakeState.AGITATE,
            new Setpoint<>()
        );


    
        public enum IntakeState {
            STOW,
            INTAKE,
            AGITATE
        }
    }

}
