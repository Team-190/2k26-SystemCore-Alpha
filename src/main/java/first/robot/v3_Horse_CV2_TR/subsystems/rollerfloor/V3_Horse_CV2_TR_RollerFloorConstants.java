package first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerConstants;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Voltage;

public class V3_Horse_CV2_TR_RollerFloorConstants {
  public static final GenericRollerConstants ROLLER_FLOOR_CONSTANTS;

  public static final Voltage ROLLER_FLOOR_RUN_VOLTAGE;
  public static final Voltage ROLLER_FLOOR_OVERRIDE_VOLTAGE;

  static {
    ROLLER_FLOOR_RUN_VOLTAGE = Volts.of(9); // TODO: Update to actual value
    ROLLER_FLOOR_OVERRIDE_VOLTAGE = Volts.of(9); // TODO: Update to actual value
    ROLLER_FLOOR_CONSTANTS =
        GenericRollerConstants.builder()
            .withLeaderCANID(31)
            .withCurrentLimits(
                CurrentLimits.builder()
                    .withSupplyCurrentLimit(Amps.of(40.0))
                    .withStatorCurrentLimit(Amps.of(40.0))
                    .build())
            .withNeutralMode(NeutralModeValue.Coast)
            .withRollerGearbox(DCMotor.getKrakenX60Foc(2))
            .withRollerMotorGearRatio(3.0 / 2.0)
            .withLeaderInvertedValue(InvertedValue.Clockwise_Positive)
            .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0001))
            .withVoltageOffsetStep(Volts.of(1))
            .withCanBus(new CANBus(CANPort.CAN_S3))
            .withEnableFOC(false)
            .withAlignedFollowerCANID(32)
            .build();
  }

  public enum RollerFloorState {
    RUN,
    OVERRIDE,
    STOP
  }
}
