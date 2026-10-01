package first.robot.v3_Horse_CV2_TR.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.ExtensionMethods;
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
import org.wpilib.units.Units;

@ExtensionMethod(ExtensionMethods.class)
public class V3_Horse_CV2_TR_Intake extends SubsystemBase {

  @Getter private ExtensionState extensionState;
  @Getter private RollerState rollerState;

  private final GenericRoller intakeRoller;

  public final Extension leftExtension;

  public final Extension rightExtension;

  private final GenericRoller kickerRoller;

  private boolean agitateIn;

  public V3_Horse_CV2_TR_Intake(
      GenericRollerIO intakeRollerIO,
      GenericRollerIO kickerRollerIO,
      ExtensionIO leftExtensionIO,
      ExtensionIO rightExtensionIO) {
    setName("Intake");

    extensionState = ExtensionState.STOW;
    rollerState = RollerState.STOP;
    agitateIn = false;

    intakeRoller =
        new GenericRoller(
            intakeRollerIO,
            this,
            V3_Horse_CV2_TR_IntakeConstants.INTAKE_ROLLER_CONSTANTS,
            "Intake Roller",
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
  }

  @Trace
  @Override
  public void periodic() {
    switch (extensionState) {
      case INTAKE, STOW:
        if (extensionStuck()) {
          leftExtension.setGainSlot(GainSlot.ONE);
          rightExtension.setGainSlot(GainSlot.ONE);
        } else {
          leftExtension.setGainSlot(GainSlot.ZERO);
          rightExtension.setGainSlot(GainSlot.ZERO);
        }
        leftExtension.setPositionGoal(
            V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
        rightExtension.setPositionGoal(
            V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_STATES.get(extensionState));

      case AGITATE:
        if (switchDirection(agitateIn)) {
          agitateIn = !agitateIn;
        }
        setAgitateGoals(agitateIn);
      case OVERRIDE:
    }
    // Rewrite agitate in periodic (not command form)
    intakeRoller.setVoltageGoal(
        V3_Horse_CV2_TR_IntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    kickerRoller.setVoltageGoal(
        V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_STATES.get(rollerState));
    intakeRoller.periodic();
    kickerRoller.periodic();
    leftExtension.periodic();
    rightExtension.periodic();
    Logger.recordOutput("Intake/Rollers/State", rollerState.toString());
    Logger.recordOutput("Intake/Extension/State", extensionState.toString());
  }

  private boolean switchDirection(boolean agitateIn) {
    ExtensionState agitateGoal;
    if (agitateIn) {
      agitateGoal = ExtensionState.AGITATE;
    } else {
      agitateGoal = ExtensionState.INTAKE;
    }
    return (((leftExtension
                    .getPositionGoal()
                    .equals(V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_STATES.get(agitateGoal))
                && leftExtension.atPositionGoal())
            && (rightExtension
                    .getPositionGoal()
                    .equals(V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_STATES.get(agitateGoal))
                && rightExtension.atPositionGoal()))
        || (leftExtension.getTorqueCurrent().abs()
            .gte(V3_Horse_CV2_TR_IntakeConstants.EXTENSION_SWITCH_CURRENT))
        || rightExtension.getTorqueCurrent().abs()
            .gte(V3_Horse_CV2_TR_IntakeConstants.EXTENSION_SWITCH_CURRENT));
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

  private boolean extensionStuck() {
    return ((leftExtension.getTorqueCurrent().abs()
                .gte(V3_Horse_CV2_TR_IntakeConstants.EXTENSION_SWITCH_CURRENT)
            && leftExtension.getVelocity().abs()
                .lte(V3_Horse_CV2_TR_IntakeConstants.EXTENSION_SWITCH_VELOCITY))
        || (rightExtension.getTorqueCurrent().abs()
                .gte(V3_Horse_CV2_TR_IntakeConstants.EXTENSION_SWITCH_CURRENT)
            && rightExtension.getVelocity().abs()
                .lte(V3_Horse_CV2_TR_IntakeConstants.EXTENSION_SWITCH_VELOCITY)));
  }

  public Command setIntakeVoltage(double voltage) {
    return Commands.runOnce(() -> intakeRoller.setVoltageGoal(Volts.of(voltage)));
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

  public Command manualExtend(DoubleSupplier getTriggerPos) {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.OVERRIDE;
          double triggerPos = getTriggerPos.getAsDouble();
          rightExtension.setPosition(
              Meters.of(
                  triggerPos
                          * (V3_Horse_CV2_TR_IntakeConstants.MAX_EXTENSION
                              - V3_Horse_CV2_TR_IntakeConstants.MIN_EXTENSION)
                      + V3_Horse_CV2_TR_IntakeConstants.MIN_EXTENSION));
          leftExtension.setPosition(
              Meters.of(
                  triggerPos
                          * (V3_Horse_CV2_TR_IntakeConstants.MAX_EXTENSION
                              - V3_Horse_CV2_TR_IntakeConstants.MIN_EXTENSION)
                      + V3_Horse_CV2_TR_IntakeConstants.MIN_EXTENSION));
        });
  }
}
