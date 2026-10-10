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

  private DoubleSupplier triggerSupplier;

  private final Setpoint<DistanceUnit> leftManualPositionGoal;

  private final Setpoint<DistanceUnit> rightManualPositionGoal;

  @Getter private Distance extensionPosition;

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

    extensionPosition = Meters.zero();
  }

  @Trace
  @Override
  public void periodic() {
    if (extensionState == ExtensionState.MANUAL_RELEASE) {
      extensionState =
          isNearStow(V3_Horse_CV2_TR_IntakeConstants.AGITATE_STOW_THRESHOLD)
              ? ExtensionState.STOW
              : ExtensionState.INTAKE;
    } else if (extensionState == ExtensionState.MANUAL_EXTEND
        && linearExtensionMap().in(Meters)
            <= V3_Horse_CV2_TR_IntakeConstants.EXTENSION_STOW_POSITION
                + V3_Horse_CV2_TR_IntakeConstants.AGITATE_AUTO_STOW_THRESHOLD
        && isNearStow(V3_Horse_CV2_TR_IntakeConstants.AGITATE_AUTO_STOW_THRESHOLD)) {
      // Pulled in to stow: latch so releasing the trigger doesn't drive it back out.
      // The goal
      // check keeps a light press from an already-stowed intake from latching
      // immediately.
      extensionState = ExtensionState.STOW;
    }

    switch (extensionState) {
      case INTAKE, STOW:
        leftExtension.setPositionGoal(
            V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
        rightExtension.setPositionGoal(
            V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_STATES.get(extensionState));
        break;
      case MANUAL_EXTEND:
        leftManualPositionGoal.setSetpoint(linearExtensionMap());
        rightManualPositionGoal.setSetpoint(linearExtensionMap());
        leftExtension.setPositionGoal(leftManualPositionGoal);
        rightExtension.setPositionGoal(rightManualPositionGoal);
        break;
      case MANUAL_RELEASE, OVERRIDE:
        break;
    }
    // if (extensionState != ExtensionState.OVERRIDE) {
    // updateGainSlots();
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

    extensionPosition = leftExtension.getExtensionPosition();

    Logger.recordOutput("Intake/Rollers/State", rollerState.toString());
    Logger.recordOutput("Intake/Extension/State", extensionState.toString());
    Logger.recordOutput("Intake/Extension/Trigger", triggerSupplier.getAsDouble());
  }

  private Distance linearExtensionMap() {
    // Blend linear with smoothstep so the ends of the trigger are finer than the
    // middle.
    // Slope is (1 - k) at the ends and (1 + k/2) in the middle.
    double t = Math.clamp(triggerSupplier.getAsDouble(), 0.0, 1.0);
    double k = V3_Horse_CV2_TR_IntakeConstants.TRIGGER_CURVE;
    double shaped = (1 - k) * t + k * t * t * (3 - 2 * t);
    Distance position =
        Meters.of(
            (1 - shaped)
                    * (V3_Horse_CV2_TR_IntakeConstants.EXTENSION_INTAKE_POSITION
                        - V3_Horse_CV2_TR_IntakeConstants.EXTENSION_STOW_POSITION)
                + V3_Horse_CV2_TR_IntakeConstants.EXTENSION_STOW_POSITION);
    return position;
  }

  // Compliant (ZERO) at rest, firmer (ONE) while driving to a new goal.
  private void updateGainSlots() {
    leftExtension.setGainSlot(leftExtension.atPositionGoal() ? GainSlot.ZERO : GainSlot.ONE);
    rightExtension.setGainSlot(rightExtension.atPositionGoal() ? GainSlot.ZERO : GainSlot.ONE);
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

  public Command setManualExtendState() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.MANUAL_EXTEND;
        });
  }

  /** Releases manual control, unless it already latched to stow. */
  public Command releaseManualExtend() {
    return Commands.runOnce(
        () -> {
          if (extensionState == ExtensionState.MANUAL_EXTEND) {
            extensionState = ExtensionState.MANUAL_RELEASE;
          }
        });
  }

  public Command zeroExtensions() {
    return Commands.sequence(
        Commands.runOnce(() -> rollerState = RollerState.STOP),
        Commands.runOnce(() -> extensionState = ExtensionState.OVERRIDE),
        Commands.parallel(leftExtension.resetExtensionZero(), rightExtension.resetExtensionZero()),
        Commands.runOnce(() -> extensionState = ExtensionState.STOW));
  }

  public Command resetIntakeZeroPosition() {
    return Commands.runOnce(
            () -> {
              extensionState = ExtensionState.OVERRIDE;
              leftExtension.setPosition(Meters.of(V3_Horse_CV2_TR_IntakeConstants.MIN_EXTENSION));
              rightExtension.setPosition(Meters.of(V3_Horse_CV2_TR_IntakeConstants.MIN_EXTENSION));
              extensionState = ExtensionState.STOW;
            })
        .ignoringDisable(true);
  }

  public Command slowMoveIn() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.OVERRIDE;
          leftExtension.setVoltageGoal(Volts.of(-1));
          rightExtension.setVoltageGoal(Volts.of(-1));
        });
  }

  public Command slowMoveOut() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.OVERRIDE;
          leftExtension.setVoltageGoal(Volts.of(1));
          rightExtension.setVoltageGoal(Volts.of(1));
        });
  }

  public Command stopExtensions() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.OVERRIDE;
          leftExtension.setVoltageGoal(Volts.zero());
          rightExtension.setVoltageGoal(Volts.zero());
        });
  }

  private boolean isNearStow(double tolerance) {
    double threshold = V3_Horse_CV2_TR_IntakeConstants.EXTENSION_STOW_POSITION + tolerance;
    return leftExtension.getExtensionPosition().in(Meters) <= threshold
        && rightExtension.getExtensionPosition().in(Meters) <= threshold;
  }

  public Command sysID() {
    return setExtensionState(ExtensionState.OVERRIDE)
        .andThen(Extension.runSysIdRoutine(this, leftExtension, rightExtension));
  }
}
