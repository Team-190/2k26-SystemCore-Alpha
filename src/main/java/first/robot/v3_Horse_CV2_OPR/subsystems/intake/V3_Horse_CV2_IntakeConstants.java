package first.robot.v3_Horse_CV2_OPR.subsystems.intake;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionConstants;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionConstants.ExtensionParameters;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import java.util.Map;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Units;

public class V3_Horse_CV2_IntakeConstants {
  public static final double INTAKE_VOLTAGE;
  public static final double EXTAKE_VOLTAGE;

  public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS;
  public static final GenericRollerConstants KICKER_ROLLER_CONSTANTS;
  public static final ExtensionConstants EXTENSION_CONSTANTS;

  public static final int MOTOR_CAN_ID = 40;
  public static final double GEAR_RATIO = 1.0 / 2.0;

  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double STATOR_CURRENT_LIMIT = 40.0;

  public static final double MOMENT_OF_INERTIA = 0.0004;
  public static final DCMotor MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);

  public static final double MAX_EXTENSION = 0.5;
  public static final double MIN_EXTENSION = 0.0;

  public static final double EXTENSION_INCREMENT = 0.0;

  public static final double EXTENSION_STOW_POSITION = 0.0;

  public static final double EXTENSION_INTAKE_POSITION = 0.25;

  public static final Map<IntakeState, Setpoint<DistanceUnit>> INTAKE_STATES;


  static {
    INTAKE_VOLTAGE = 12.0;
    EXTAKE_VOLTAGE = -12.0;

    INTAKE_ROLLER_CONSTANTS =
        GenericRollerConstants.builder()
            .withLeaderCANID(40) // not sure if these are taken or not
            .withCurrentLimits(
                CurrentLimits.builder()
                    .withSupplyCurrentLimit(Amps.of(40.0))
                    .withStatorCurrentLimit(Amps.of(40.0))
                    .build())
            .withNeutralMode(NeutralModeValue.Coast)
            .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
            .withRollerMotorGearRatio((1.0 / 2.0)) // please god someone change this
            .withLeaderInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withOpposedFollowerCANID(41)
            .withMomentOfInertia(Units.KilogramSquareMeters.of(MOMENT_OF_INERTIA))
            .withVoltageOffsetStep(Volts.of(1))
            .withCanBus(CANBus.systemcore(0))
            .withEnableFOC(false)
            .build();
    KICKER_ROLLER_CONSTANTS =
        GenericRollerConstants.builder()
            .withLeaderCANID(42) // not sure if these are taken or not
            .withCurrentLimits(
                CurrentLimits.builder()
                    .withSupplyCurrentLimit(Amps.of(40.0))
                    .withStatorCurrentLimit(Amps.of(40.0))
                    .build())
            .withNeutralMode(NeutralModeValue.Coast)
            .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
            .withRollerMotorGearRatio((1.0 / 2.0)) // same here, please change this
            .withLeaderInvertedValue(InvertedValue.CounterClockwise_Positive)
            .withOpposedFollowerCANID(41)
            .withMomentOfInertia(Units.KilogramSquareMeters.of(MOMENT_OF_INERTIA))
            .withVoltageOffsetStep(Volts.of(1))
            .withCanBus(CANBus.systemcore(0))
            .withEnableFOC(false)
            .build();
    
    EXTENSION_CONSTANTS =
        ExtensionConstants.builder()
            .withLeaderCANID(44)
            .withExtensionGearRatio(null) //and here
            .withDrumRadius(0.025)
            .withExtensionSupplyCurrentLimit(40.0)
            .withExtensionStatorCurrentLimit(40.0)
            .withSlot0Gains(Gains.builder().build())
            .withSlot1Gains(Gains.builder().build())
            .withSlot2Gains(Gains.builder().build())
            .withVerticalGravity(false)
            .withAlignedFollowerCANID(45)
            .withOpposedFollowerCANID(46)
            .withVoltageOffsetStep(Volts.of(1))
            .withHeightOffsetStep(Meters.of(0.01))
            .build();   

    INTAKE_STATES =
        Map.of(
            IntakeState.STOW,
            new Setpoint<>(
                Meters.of(EXTENSION_STOW_POSITION),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)),
            IntakeState.INTAKE,
            new Setpoint<>(
                Meters.of(EXTENSION_INTAKE_POSITION),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)),
            IntakeState.AGITATE,
            new Setpoint<>(
                Meters.of(0.25),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)));
  }

  public enum IntakeState {
    STOW,
    INTAKE,
    AGITATE
  }
}
