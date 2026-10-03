package first.robot.v3_Horse_CV2_TR.subsystems.intake;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.ExtensionMethods;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.extension.Extension;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants.ExtensionState;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants.RollerState;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.measure.Distance;

@ExtensionMethod(ExtensionMethods.class)
public class V3_Horse_CV2_TR_Intake extends SubsystemBase {

  @Getter private ExtensionState extensionState;
  @Getter private RollerState rollerState;

  private final GenericRoller leftIntakeRoller;
  private final GenericRoller rightIntakeRoller;

  public final Extension leftExtension;

  public final Extension rightExtension;

  private final GenericRoller kickerRoller;

  private boolean agitateIn;

  private DoubleSupplier triggerSupplier;

  private final Setpoint<DistanceUnit> leftManualPositionGoal;

  private final Setpoint<DistanceUnit> rightManualPositionGoal;

  public V3_Horse_CV2_TR_Intake(
      GenericRollerIO leftIntakeRollerIO,
      GenericRollerIO rightIntakeRollerIO,
      GenericRollerIO kickerRollerIO,
      ExtensionIO leftExtensionIO,
      ExtensionIO rightExtensionIO,
      DoubleSupplier triggerSupplier) {
    setName("Intake");

    extensionState = ExtensionState.STOW;
    rollerState = RollerState.STOP;
    agitateIn = false;
    this.triggerSupplier = triggerSupplier;

    leftIntakeRoller =
        new GenericRoller(
            leftIntakeRollerIO,
            this,
            V3_Horse_CV2_TR_IntakeConstants.LEFT_INTAKE_ROLLER_CONSTANTS,
            "Left Intake Roller",
            V3_Horse_CV2_TR_IntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    rightIntakeRoller =
        new GenericRoller(
            rightIntakeRollerIO,
            this,
            V3_Horse_CV2_TR_IntakeConstants.RIGHT_INTAKE_ROLLER_CONSTANTS,
            "Right Intake Roller",
            V3_Horse_CV2_TR_IntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));

    kickerRoller =
        new GenericRoller(
            kickerRollerIO,
            this,
            V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_CONSTANTS,
            "Kicker Roller",
            V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_STATES.get(rollerState));

    leftExtension =
        new Extension(
            V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_CONSTANTS,
            this,
            0,
            leftExtensionIO,
            V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
    rightExtension =
        new Extension(
            V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_CONSTANTS,
            this,
            1,
            rightExtensionIO,
            V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_STATES.get(extensionState));

    leftManualPositionGoal =
        new Setpoint<>(
            Meters.of(V3_Horse_CV2_TR_IntakeConstants.EXTENSION_STOW_POSITION),
            Meters.of(0.01),
            Meters.of(V3_Horse_CV2_TR_IntakeConstants.MIN_EXTENSION),
            Meters.of(V3_Horse_CV2_TR_IntakeConstants.MAX_EXTENSION));
    rightManualPositionGoal =
        new Setpoint<>(
            Meters.of(V3_Horse_CV2_TR_IntakeConstants.EXTENSION_STOW_POSITION),
            Meters.of(0.01),
            Meters.of(V3_Horse_CV2_TR_IntakeConstants.MIN_EXTENSION),
            Meters.of(V3_Horse_CV2_TR_IntakeConstants.MAX_EXTENSION));
  }

  @Trace
  @Override
  public void periodic() {
    switch (extensionState) {
      case INTAKE, STOW:
        leftExtension.setPositionGoal(
            V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
        rightExtension.setPositionGoal(
            V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_STATES.get(extensionState));
        break;
      case AGITATE:
        if (switchDirection(agitateIn)) {
          agitateIn = !agitateIn;
        }
        setAgitateGoals(agitateIn);
        break;
      case MANUAL_EXTEND:
        if (triggerSupplier.getAsDouble() < 0.9) {
          leftManualPositionGoal.setSetpoint(linearExtensionMap());
          rightManualPositionGoal.setSetpoint(linearExtensionMap());
          leftExtension.setPositionGoal(leftManualPositionGoal);
          rightExtension.setPositionGoal(rightManualPositionGoal);
        } else {
          extensionState = ExtensionState.STOW;
        }
        break;
      case OVERRIDE:
        break;
    }
    // if (extensionState != ExtensionState.OVERRIDE) {
    //   updateGainSlots();
    // }
    leftIntakeRoller.setVoltageGoal(
        V3_Horse_CV2_TR_IntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    rightIntakeRoller.setVoltageGoal(
        V3_Horse_CV2_TR_IntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    kickerRoller.setVoltageGoal(
        V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_STATES.get(rollerState));
    leftIntakeRoller.periodic();
    rightIntakeRoller.periodic();
    kickerRoller.periodic();
    leftExtension.periodic();
    rightExtension.periodic();
    Logger.recordOutput("Intake/Rollers/State", rollerState.toString());
    Logger.recordOutput("Intake/Extension/State", extensionState.toString());
  }

  private Distance linearExtensionMap() {
    Distance position =
        Meters.of(
            (1 - triggerSupplier.getAsDouble())
                    * (V3_Horse_CV2_TR_IntakeConstants.EXTENSION_INTAKE_POSITION
                        - V3_Horse_CV2_TR_IntakeConstants.EXTENSION_STOW_POSITION)
                + V3_Horse_CV2_TR_IntakeConstants.EXTENSION_STOW_POSITION);
    return position;
  }

  private boolean switchDirection(boolean agitateIn) {
    ExtensionState agitateGoal;
    if (agitateIn) {
      agitateGoal = ExtensionState.AGITATE;
    } else {
      agitateGoal = ExtensionState.INTAKE;
    }
    return ((leftExtension
                    .getPositionGoal()
                    .getNewSetpoint()
                    .matchesSetpoint(
                        V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_STATES.get(agitateGoal))
                && leftExtension.atPositionGoal())
            && (rightExtension
                    .getPositionGoal()
                    .getNewSetpoint()
                    .matchesSetpoint(
                        V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_STATES.get(agitateGoal))
                && rightExtension.atPositionGoal()))
        || (leftExtension
            .getTorqueCurrent()
            .abs()
            .gte(V3_Horse_CV2_TR_IntakeConstants.EXTENSION_SWITCH_CURRENT))
        || rightExtension
            .getTorqueCurrent()
            .abs()
            .gte(V3_Horse_CV2_TR_IntakeConstants.EXTENSION_SWITCH_CURRENT);
  }

  private void setAgitateGoals(boolean agitateIn) {
    ExtensionState agitateGoal;
    if (agitateIn) {
      agitateGoal = ExtensionState.AGITATE;
    } else {
      agitateGoal = ExtensionState.INTAKE;
    }

    leftExtension.setPositionGoal(
        V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_STATES.get(agitateGoal));
    rightExtension.setPositionGoal(
        V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_STATES.get(agitateGoal));
  }

  // Compliant (ZERO) at rest, firmer (ONE) while driving to a new goal.
  private void updateGainSlots() {
    leftExtension.setGainSlot(leftExtension.atPositionGoal() ? GainSlot.ZERO : GainSlot.ONE);
    rightExtension.setGainSlot(rightExtension.atPositionGoal() ? GainSlot.ZERO : GainSlot.ONE);
  }

  public Command setIntakeVoltage(double voltage) {
    return Commands.runOnce(
        () -> {
          leftIntakeRoller.setVoltageGoal(Volts.of(voltage));
          rightIntakeRoller.setVoltageGoal(Volts.of(voltage));
        });
  }

  public Command setKickerVoltage(double voltage) {
    return Commands.runOnce(() -> kickerRoller.setVoltageGoal(Volts.of(voltage)));
  }

  public Command deploy() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.INTAKE;
          rollerState = RollerState.INTAKE;
        });
  }

  public Command stowAndStop() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.STOW;
          rollerState = RollerState.STOP;
        });
  }

  public Command stopRollers() {
    return Commands.runOnce(
        () -> {
          rollerState = RollerState.STOP;
        });
  }

  public Command intakeRollers() {
    return Commands.runOnce(
        () -> {
          rollerState = RollerState.INTAKE;
        });
  }

  public Command extakeRollers() {
    return Commands.runOnce(
        () -> {
          rollerState = RollerState.EXTAKE;
        });
  }

  public Command extend() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.INTAKE;
        });
  }

  public Command retract() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.STOW;
        });
  }

  public Command stowExtension() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.STOW;
        });
  }

  public Command setExtensionState(ExtensionState state) {
    return Commands.runOnce(
        () -> {
          extensionState = state;
        });
  }

  public Command setRollerState(RollerState state) {
    return Commands.runOnce(
        () -> {
          rollerState = state;
        });
  }

  public Command setIntakeState(ExtensionState e_state, RollerState r_state) {
    return Commands.runOnce(
        () -> {
          extensionState = e_state;
          rollerState = r_state;
        });
  }

  public Command setManualExtendState() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.MANUAL_EXTEND;
        });
  }

  public Command sysID() {
    return Commands.runOnce(
            () -> {
              extensionState = ExtensionState.OVERRIDE;
            })
        .andThen(
            Commands.parallel(leftExtension.runSysIdRoutine(), rightExtension.runSysIdRoutine()));
  }
}
