package first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.FollowerMotor;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.NeutralMode;
import edu.wpi.team190.gompeilib.io.motor.config.OpenLoopRamps;
import edu.wpi.team190.gompeilib.robot.mechanisms.roller.GenericRollerConstants;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Voltage;

public class V3_Horse_CV2_TRRollerFloorConstants {
  public static final GenericRollerConstants ROLLER_FLOOR_CONSTANTS;

  public static final Voltage ROLLER_FLOOR_RUN_VOLTAGE;
  public static final Voltage ROLLER_FLOOR_OVERRIDE_VOLTAGE;
  public static final Voltage ROLLER_FLOOR_SLOW_VOLTAGE;

  static {
    ROLLER_FLOOR_RUN_VOLTAGE = Volts.of(12); // TODO: Update to actual value
    ROLLER_FLOOR_OVERRIDE_VOLTAGE = Volts.of(12); // TODO: Update to actual value
    ROLLER_FLOOR_SLOW_VOLTAGE = Volts.of(6);
    ROLLER_FLOOR_CONSTANTS =
        GenericRollerConstants.builder()
            .withMotor(
                MotorConfig.builder()
                    .withCanId(31)
                    .withBus(CANBusId.port(CANPort.CAN_S3))
                    .withDirection(MotorDirection.CLOCKWISE_POSITIVE)
                    .withNeutralMode(NeutralMode.COAST)
                    .withCurrentLimits(
                        CurrentLimits.builder()
                            .withSupplyCurrentLimit(Amps.of(40.0))
                            .withStatorCurrentLimit(Amps.of(40.0))
                            .build())
                    .withGearRatio(84.0 / 16)
                    .withOpenLoopRamps(
                        new OpenLoopRamps(Seconds.of(0.0), Seconds.of(0.25), Seconds.of(0.0)))
                    .withEnableFOC(false)
                    .withFollower(new FollowerMotor(32, MotorAlignment.ALIGNED))
                    .build())
            .withRollerGearbox(DCMotor.getKrakenX60Foc(2))
            .withMomentOfInertia(Units.KilogramSquareMeters.of(0.0001))
            .withVoltageOffsetStep(Volts.of(1))
            .build();
  }

  public enum RollerFloorState {
    RUN,
    OVERRIDE,
    STOP
  }
}
