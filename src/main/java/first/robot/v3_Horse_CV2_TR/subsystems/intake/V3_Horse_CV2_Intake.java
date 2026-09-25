package first.robot.v3_Horse_CV2_TR.subsystems.intake;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.extension.Extension;
import edu.wpi.team190.gompeilib.subsystems.extension.ExtensionIO;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_IntakeConstants.ExtensionState;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_IntakeConstants.RollerState;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.util.function.BooleanConsumer;

public class V3_Horse_CV2_Intake extends SubsystemBase {

  @Getter private ExtensionState extensionState;
  @Getter private RollerState rollerState;

  private final GenericRoller intakeRoller;

  public final Extension extension;

  private final GenericRoller kickerRoller;

  public record IntakeStateSetter(
      BooleanConsumer stowed,
      BooleanConsumer collecting,
      BooleanConsumer spitting,
      BooleanConsumer slowCollecting) {
    public IntakeStateSetter() {
      this(b -> {}, b -> {}, b -> {}, b -> {});
    }
  }

  public V3_Horse_CV2_Intake(
      GenericRollerIO intakeRollerIO, GenericRollerIO kickerRollerIO, ExtensionIO extensionIO) {
    setName("Intake");

    extensionState = ExtensionState.STOW;
    rollerState = RollerState.STOP;

    intakeRoller =
        new GenericRoller(
            intakeRollerIO,
            this,
            V3_Horse_CV2_IntakeConstants.INTAKE_ROLLER_CONSTANTS,
            "",
            V3_Horse_CV2_IntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));

    kickerRoller =
        new GenericRoller(
            kickerRollerIO,
            this,
            V3_Horse_CV2_IntakeConstants.KICKER_ROLLER_CONSTANTS,
            "",
            V3_Horse_CV2_IntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));

    extension =
        new Extension(
            V3_Horse_CV2_IntakeConstants.EXTENSION_CONSTANTS,
            this,
            3,
            extensionIO,
            V3_Horse_CV2_IntakeConstants.EXTENSION_STATES.get(extensionState));
  }

  @Trace
  @Override
  public void periodic() {
    if (!(extensionState.equals(ExtensionState.AGITATE))) {
      extension.setPositionGoal(V3_Horse_CV2_IntakeConstants.EXTENSION_STATES.get(extensionState));
    }
    // Rewrite agitate in periodic (not command form)
    intakeRoller.setVoltageGoal(V3_Horse_CV2_IntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    kickerRoller.setVoltageGoal(V3_Horse_CV2_IntakeConstants.KICKER_ROLLER_STATES.get(rollerState));
    intakeRoller.periodic();
    kickerRoller.periodic();
    extension.periodic();
    Logger.recordOutput("Intake/Rollers/State", rollerState.toString());
    Logger.recordOutput("Intake/Extension/State", extensionState.toString());
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

  public Command agitate() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  extensionState = ExtensionState.AGITATE;
                  extension.setPositionGoal(
                      V3_Horse_CV2_IntakeConstants.EXTENSION_STATES.get(ExtensionState.AGITATE));
                }),
            extension
                .waitUntilAtGoal()
                .until(() -> extension.getTorqueCurrent().isNear(Amps.of(35), Milliamps.of(500))),
            Commands.runOnce(
                () -> {
                  extension.setPositionGoal(extension.getExtensionPosition().minus(Meters.of(1)));
                }),
            extension
                .waitUntilAtGoal()
                .until(() -> extension.getTorqueCurrent().isNear(Amps.of(-45), Milliamps.of(500))))
        .repeatedly();
  }
}
