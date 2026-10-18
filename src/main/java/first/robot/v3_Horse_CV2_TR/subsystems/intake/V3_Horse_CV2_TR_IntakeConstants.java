package first.robot.v3_Horse_CV2_TR.subsystems.intake;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.NeutralMode;
import edu.wpi.team190.gompeilib.io.motor.config.OpenLoopRamps;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionConstants;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionConstants.ExtensionParameters;
import edu.wpi.team190.gompeilib.subsystems.roller.GenericRollerConstants;
import java.util.Map;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Units;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

public class V3_Horse_CV2_TR_IntakeConstants {

  public static final GenericRollerConstants LEFT_INTAKE_ROLLER_CONSTANTS;
  public static final GenericRollerConstants RIGHT_INTAKE_ROLLER_CONSTANTS;
  public static final GenericRollerConstants KICKER_ROLLER_CONSTANTS;
  public static final ExtensionConstants LEFT_EXTENSION_CONSTANTS;
  public static final ExtensionConstants RIGHT_EXTENSION_CONSTANTS;

  public static final int MOTOR_CAN_ID = 40;
  public static final double GEAR_RATIO = 1.0 / 2.0;

  public static final double SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double STATOR_CURRENT_LIMIT = 40.0;

  public static final double MOMENT_OF_INERTIA = 0.0004;
  public static final DCMotor MOTOR_CONFIG = DCMotor.getKrakenX60Foc(1);

  public static final Distance EXTENSION_DRUM_RADIUS = Meters.of(0.0127);

  public static final double MAX_EXTENSION = 0.3;
  public static final double MIN_EXTENSION = 0.0;

  public static final double EXTENSION_INCREMENT = 0.0;

  public static final double EXTENSION_STOW_POSITION = 0.0;

  public static final double EXTENSION_INTAKE_POSITION = 0.3;

  // Releasing agitate within this distance of stow stows the intake
  public static final double AGITATE_STOW_THRESHOLD = 0.05; // TODO: tune

  // While agitating, reaching within this distance of stow latches the intake to stow
  public static final double AGITATE_AUTO_STOW_THRESHOLD = 0.01;

  // Trigger-to-extension curve, 0 = linear, 1 = full S-curve (flat at the ends)
  public static final double TRIGGER_CURVE = 0;

  public static final Map<ExtensionState, Setpoint<DistanceUnit>> LEFT_EXTENSION_STATES;

  public static final Map<ExtensionState, Setpoint<DistanceUnit>> RIGHT_EXTENSION_STATES;

  public static final Map<RollerState, Setpoint<VoltageUnit>> INTAKE_ROLLER_STATES;

  public static final Map<RollerState, Setpoint<VoltageUnit>> KICKER_ROLLER_STATES;

  public static final Voltage INTAKE_IN_ROLLER_VOLTAGE;

  public static final Voltage KICKER_IN_ROLLER_VOLTAGE;

  public static final Voltage INTAKE_OUT_ROLLER_VOLTAGE;

  public static final Voltage KICKER_OUT_ROLLER_VOLTAGE;

  public static final Voltage IDLE_ROLLER_VOLTAGE;

  static {
    INTAKE_IN_ROLLER_VOLTAGE = Volts.of(11.0);
    INTAKE_OUT_ROLLER_VOLTAGE = Volts.of(-12.0);
    KICKER_IN_ROLLER_VOLTAGE = Volts.of(11.0);
    KICKER_OUT_ROLLER_VOLTAGE = Volts.of(-12.0);
    IDLE_ROLLER_VOLTAGE = Volts.of(4.0); // TODO: tune

    LEFT_INTAKE_ROLLER_CONSTANTS =
        rollerConstants(
            23,
            CANBusId.port(CANPort.CAN_S2),
            MotorDirection.COUNTER_CLOCKWISE_POSITIVE,
            36.0 / 12);
    RIGHT_INTAKE_ROLLER_CONSTANTS =
        rollerConstants(
            24, CANBusId.port(CANPort.CAN_S1), MotorDirection.CLOCKWISE_POSITIVE, 36.0 / 12);
    KICKER_ROLLER_CONSTANTS =
        rollerConstants(
            25,
            CANBusId.named("Extension"),
            MotorDirection.COUNTER_CLOCKWISE_POSITIVE,
            (58.0 / 26) * (24.0 / 18));

    LEFT_EXTENSION_CONSTANTS =
        extensionConstants(21, MotorDirection.COUNTER_CLOCKWISE_POSITIVE, "Left Extension");
    RIGHT_EXTENSION_CONSTANTS =
        extensionConstants(22, MotorDirection.CLOCKWISE_POSITIVE, "Right Extension");

    LEFT_EXTENSION_STATES =
        Map.of(
            ExtensionState.STOW,
            new Setpoint<>(
                Meters.of(EXTENSION_STOW_POSITION),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)),
            ExtensionState.INTAKE,
            new Setpoint<>(
                Meters.of(EXTENSION_INTAKE_POSITION),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)));

    RIGHT_EXTENSION_STATES =
        Map.of(
            ExtensionState.STOW,
            new Setpoint<>(
                Meters.of(EXTENSION_STOW_POSITION),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)),
            ExtensionState.INTAKE,
            new Setpoint<>(
                Meters.of(EXTENSION_INTAKE_POSITION),
                Meters.of(0.01),
                Meters.of(MIN_EXTENSION),
                Meters.of(MAX_EXTENSION)));

    INTAKE_ROLLER_STATES =
        Map.of(
            RollerState.INTAKE,
            new Setpoint<>(
                INTAKE_IN_ROLLER_VOLTAGE,
                LEFT_INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.EXTAKE,
            new Setpoint<>(
                INTAKE_OUT_ROLLER_VOLTAGE,
                LEFT_INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.IDLE,
            new Setpoint<>(
                IDLE_ROLLER_VOLTAGE,
                LEFT_INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.STOP,
            new Setpoint<>(
                Volts.of(0),
                LEFT_INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)));

    KICKER_ROLLER_STATES =
        Map.of(
            RollerState.INTAKE,
            new Setpoint<>(
                KICKER_IN_ROLLER_VOLTAGE,
                KICKER_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.EXTAKE,
            new Setpoint<>(
                KICKER_OUT_ROLLER_VOLTAGE,
                KICKER_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.IDLE,
            new Setpoint<>(
                IDLE_ROLLER_VOLTAGE,
                KICKER_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.STOP,
            new Setpoint<>(
                Volts.of(0),
                KICKER_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)));
  }

  /** Converts a per-meter extension gain to per drum rotation, the unit the motor works in. */
  private static double perDrumRotation(double perMeter) {
    return perMeter * 2.0 * Math.PI * EXTENSION_DRUM_RADIUS.in(Meters);
  }

  /** The two extensions share everything but their ID, direction, and tunable prefix. */
  private static ExtensionConstants extensionConstants(
      int canId, MotorDirection direction, String name) {
    return ExtensionConstants.builder()
        .withMotor(
            MotorConfig.builder()
                .withCanId(canId)
                .withBus(CANBusId.named("Extension"))
                .withDirection(direction)
                .withNeutralMode(NeutralMode.COAST)
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(40.0))
                        .withStatorCurrentLimit(Amps.of(40.0))
                        .build())
                .withGearRatio((52.0 / 30) * (52.0 / 16))
                .withSlot(
                    GainSlot.ZERO,
                    new SlotConfig(
                        Gains.builder()
                            .withPrefix("Intake/" + name + "/Slot0")
                            .withKP(perDrumRotation(10))
                            .withKD(0.0)
                            .withKS(0.42002)
                            .withKV(perDrumRotation(6.3697))
                            .withKA(0.0)
                            .build()))
                .withSlot(
                    GainSlot.ONE,
                    new SlotConfig(
                        Gains.builder()
                            .withPrefix("Intake/" + name + "/Slot1")
                            .withKP(perDrumRotation(1.0))
                            .withKD(0.0)
                            .withKS(0.0)
                            .withKV(0.0)
                            .withKA(0.0)
                            .build()))
                .build())
        .withDrumRadius(EXTENSION_DRUM_RADIUS)
        .withVoltageOffsetStep(Volts.of(1))
        .withHeightOffsetStep(Meters.of(0.01))
        .withExtensionParameters(
            ExtensionParameters.builder()
                .withCarriageMass(Kilograms.of(10.0))
                .withCarriageGearbox(DCMotor.getKrakenX44Foc(1))
                .withMaxLength(Meters.of(MAX_EXTENSION))
                .withMinLength(Meters.of(MIN_EXTENSION))
                .withSimulateGravity(false)
                .build())
        .withConstraints(
            LinearConstraints.builder()
                .withPrefix("Intake/" + name + "/Constraints")
                .withGoalTolerance(Meters.of(0.01))
                .withMaxAcceleration(MetersPerSecondPerSecond.of(10))
                .withMaxVelocity(MetersPerSecond.of(20))
                .build())
        .build();
  }

  /** The intake and kicker rollers share everything but their ID, bus, direction, and ratio. */
  private static GenericRollerConstants rollerConstants(
      int canId, CANBusId bus, MotorDirection direction, double gearRatio) {
    return GenericRollerConstants.builder()
        .withMotor(
            MotorConfig.builder()
                .withCanId(canId)
                .withBus(bus)
                .withDirection(direction)
                .withNeutralMode(NeutralMode.COAST)
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(40.0))
                        .withStatorCurrentLimit(Amps.of(40.0))
                        .build())
                .withGearRatio(gearRatio)
                .withOpenLoopRamps(
                    new OpenLoopRamps(Seconds.of(0.0), Seconds.of(0.25), Seconds.of(0.0)))
                .withEnableFOC(false)
                .build())
        .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
        .withMomentOfInertia(Units.KilogramSquareMeters.of(MOMENT_OF_INERTIA))
        .withVoltageOffsetStep(Volts.of(1))
        .build();
  }

  public enum ExtensionState {
    STOW,
    INTAKE,
    MANUAL_EXTEND,
    MANUAL_RELEASE,
    OVERRIDE
  }

  public enum RollerState {
    INTAKE,
    IDLE,
    EXTAKE,
    STOP
  }
}
