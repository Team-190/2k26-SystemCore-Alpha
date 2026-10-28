package first.robot.v3_Horse.subsystems.intake;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.FollowerMotor;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.NeutralMode;
import edu.wpi.team190.gompeilib.io.motor.config.OpenLoopRamps;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.extension.ExtensionConstants;
import edu.wpi.team190.gompeilib.robot.mechanisms.extension.ExtensionConstants.ExtensionParameters;
import edu.wpi.team190.gompeilib.robot.mechanisms.roller.GenericRollerConstants;
import java.util.Map;
import java.util.Optional;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Units;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

public class V3_HorseIntakeConstants {

  public static final GenericRollerConstants INTAKE_ROLLER_CONSTANTS;
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

  public static final Voltage EXTENSION_SLOW_MOVE_VOLTAGE;

  static {
    INTAKE_IN_ROLLER_VOLTAGE = Volts.of(11.0);
    INTAKE_OUT_ROLLER_VOLTAGE = Volts.of(-12.0);
    KICKER_IN_ROLLER_VOLTAGE = Volts.of(11.0);
    KICKER_OUT_ROLLER_VOLTAGE = Volts.of(-12.0);
    IDLE_ROLLER_VOLTAGE = Volts.of(5.0); // TODO: tune
    EXTENSION_SLOW_MOVE_VOLTAGE = Volts.of(1.0);

    // The right roller is on another bus, so it mirrors the left roller instead of following it.
    INTAKE_ROLLER_CONSTANTS =
        GenericRollerConstants.builder()
            .withMotor(
                MotorConfig.builder()
                    .withCanId(23)
                    .withBus(CANBusId.port(CANPort.CAN_S2))
                    .withDirection(MotorDirection.COUNTER_CLOCKWISE_POSITIVE)
                    .withNeutralMode(NeutralMode.COAST)
                    .withCurrentLimits(
                        CurrentLimits.builder()
                            .withSupplyCurrentLimit(Amps.of(40.0))
                            .withStatorCurrentLimit(Amps.of(40.0))
                            .build())
                    .withGearRatio(36.0 / 12)
                    .withOpenLoopRamps(
                        new OpenLoopRamps(Seconds.of(0.0), Seconds.of(0.25), Seconds.of(0.0)))
                    .withEnableFOC(false)
                    .withFollower(
                        new FollowerMotor(24, MotorAlignment.OPPOSED)
                            .onBus(CANBusId.port(CANPort.CAN_S1)))
                    .build())
            .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
            .withPlant(
                PlantCharacterization.ofInertia(
                        DCMotor.getKrakenX60Foc(1),
                        36.0 / 12,
                        Units.KilogramSquareMeters.of(MOMENT_OF_INERTIA))
                    .build())
            .withVoltageOffsetStep(Volts.of(1))
            .build();
    KICKER_ROLLER_CONSTANTS =
        GenericRollerConstants.builder()
            .withMotor(
                MotorConfig.builder()
                    .withCanId(25)
                    .withBus(CANBusId.port(CANPort.CAN_S0))
                    .withDirection(MotorDirection.COUNTER_CLOCKWISE_POSITIVE)
                    .withNeutralMode(NeutralMode.COAST)
                    .withCurrentLimits(
                        CurrentLimits.builder()
                            .withSupplyCurrentLimit(Amps.of(40.0))
                            .withStatorCurrentLimit(Amps.of(40.0))
                            .build())
                    .withGearRatio((58.0 / 26) * (24.0 / 18))
                    .withOpenLoopRamps(
                        new OpenLoopRamps(Seconds.of(0.0), Seconds.of(0.25), Seconds.of(0.0)))
                    .withEnableFOC(false)
                    .build())
            .withRollerGearbox(DCMotor.getKrakenX60Foc(1))
            .withPlant(
                PlantCharacterization.ofInertia(
                        DCMotor.getKrakenX60Foc(1),
                        (58.0 / 26) * (24.0 / 18),
                        Units.KilogramSquareMeters.of(MOMENT_OF_INERTIA))
                    .build())
            .withVoltageOffsetStep(Volts.of(1))
            .build();

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
                INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.EXTAKE,
            new Setpoint<>(
                INTAKE_OUT_ROLLER_VOLTAGE,
                INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.IDLE,
            new Setpoint<>(
                IDLE_ROLLER_VOLTAGE,
                INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
                Volts.of(-12.0),
                Volts.of(12.0)),
            RollerState.STOP,
            new Setpoint<>(
                Volts.of(0),
                INTAKE_ROLLER_CONSTANTS.voltageOffsetStep,
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

  /** The two extensions share everything but their ID, direction, and tunable prefix. */
  private static ExtensionConstants extensionConstants(
      int canId, MotorDirection direction, String name) {
    return ExtensionConstants.builder()
        .withMotor(
            MotorConfig.builder()
                .withCanId(canId)
                .withBus(CANBusId.port(CANPort.CAN_S0))
                .withDirection(direction)
                .withNeutralMode(NeutralMode.COAST)
                .withCurrentLimits(
                    CurrentLimits.builder()
                        .withSupplyCurrentLimit(Amps.of(40.0))
                        .withStatorCurrentLimit(Amps.of(40.0))
                        .build())
                .withGearRatio((52.0 / 30) * (52.0 / 16))
                .build())
        .withSlot(
            GainSlot.ZERO,
            Gains.linearPosition(Volts)
                .withPrefix("Intake/" + name + "/Slot0")
                .withKP(VoltsPerMeter.ofNative(10))
                .withKS(Volts.of(0.42002))
                .withKV(VoltsPerMeterPerSecond.ofNative(6.3697))
                .build())
        .withSlot(
            GainSlot.ONE,
            Gains.linearPosition(Volts)
                .withPrefix("Intake/" + name + "/Slot1")
                .withKP(VoltsPerMeter.ofNative(0.1))
                .build())
        .withDrumRadius(EXTENSION_DRUM_RADIUS)
        // Drive back into the hard stop at 1 V until the torque current reaches -20 A
        .withHoming(
            Optional.of(
                new CurrentHoming.Config<>(
                    Volts.of(-1.0), Amps.of(-20.0), Amps.of(5.0), Meters.of(MIN_EXTENSION))))
        .withVoltageOffsetStep(Volts.of(1))
        .withHeightOffsetStep(Meters.of(0.01))
        .withExtensionParameters(
            ExtensionParameters.builder()
                .withCarriageGearbox(DCMotor.getKrakenX44Foc(1))
                .withMaxLength(Meters.of(MAX_EXTENSION))
                .withMinLength(Meters.of(MIN_EXTENSION))
                // From CAD; the extension is horizontal, so gravity doesn't load it
                .withPlant(
                    PlantCharacterization.ofElevator(
                            DCMotor.getKrakenX44Foc(1),
                            (52.0 / 30) * (52.0 / 16),
                            Kilograms.of(10.0),
                            EXTENSION_DRUM_RADIUS)
                        .withKG(Volts.of(0))
                        .build())
                .build())
        .withConstraints(
            LinearConstraints.builder()
                .withPrefix("Intake/" + name + "/Constraints")
                .withGoalTolerance(Meters.of(0.01))
                .withMaxAcceleration(MetersPerSecondPerSecond.of(10))
                .withMaxVelocity(MetersPerSecond.of(20))
                .build())
        .withCharacterization(
            CharacterizationConfig.voltage()
                .withPrefix("Intake/" + name + "/Characterization")
                .withRampRate(VoltsPerSecond.ofNative(1))
                .withStepOutput(Volts.of(3))
                .withQuasistaticTimeout(Seconds.of(3))
                .withDynamicTimeout(Seconds.of(3))
                .withSettleTime(Seconds.of(1))
                .build())
        .build();
  }

  public enum ExtensionState {
    STOW,
    INTAKE,
    MANUAL_EXTEND,
    ZERO,
    STOP,
    SLOW_MOVE,
    SQUISH,
    SYSID
  }

  public enum RollerState {
    INTAKE,
    IDLE,
    EXTAKE,
    STOP
  }
}
