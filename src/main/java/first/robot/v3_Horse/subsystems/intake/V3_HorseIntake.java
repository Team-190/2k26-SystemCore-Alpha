package first.robot.v3_Horse.subsystems.intake;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.characterization.CharacterizationRoutine;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.mechanisms.extension.Extension;
import edu.wpi.team190.gompeilib.robot.mechanisms.roller.GenericRoller;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.ExtensionState;
import first.robot.v3_Horse.subsystems.intake.V3_HorseIntakeConstants.RollerState;
import java.util.List;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.measure.Distance;

public class V3_HorseIntake extends SubsystemBase {

  @Getter private ExtensionState extensionState;
  @Getter private RollerState rollerState;
  private boolean characterizing = false;

  private final GenericRoller intakeRoller;

  public final Extension leftExtension;

  public final Extension rightExtension;

  private final GenericRoller kickerRoller;

  private DoubleSupplier triggerSupplier;

  private final Setpoint<DistanceUnit> leftManualPositionGoal;

  private final Setpoint<DistanceUnit> rightManualPositionGoal;

  @Getter private Distance extensionPosition;

  private boolean manualReachedStow;

  private ExtensionState previousExtensionState;
  private boolean leftCompliant;
  private boolean rightCompliant;
  private GainSlot leftGainSlot;
  private GainSlot rightGainSlot;

  public V3_HorseIntake(
      MotorIOFactory intakeRollerMotors,
      MotorIOFactory kickerRollerMotors,
      MotorIOFactory leftExtensionMotors,
      MotorIOFactory rightExtensionMotors,
      DoubleSupplier triggerSupplier) {
    setName("Intake");

    extensionState = ExtensionState.STOW;
    rollerState = RollerState.STOP;
    this.triggerSupplier = triggerSupplier;

    intakeRoller =
        new GenericRoller(
            intakeRollerMotors,
            this,
            V3_HorseIntakeConstants.INTAKE_ROLLER_CONSTANTS,
            "IntakeRoller",
            V3_HorseIntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));

    kickerRoller =
        new GenericRoller(
            kickerRollerMotors,
            this,
            V3_HorseIntakeConstants.KICKER_ROLLER_CONSTANTS,
            "KickerRoller",
            V3_HorseIntakeConstants.KICKER_ROLLER_STATES.get(rollerState));

    leftExtension =
        new Extension(
            leftExtensionMotors,
            this,
            "LeftExtension",
            V3_HorseIntakeConstants.LEFT_EXTENSION_CONSTANTS,
            V3_HorseIntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
    rightExtension =
        new Extension(
            rightExtensionMotors,
            this,
            "RightExtension",
            V3_HorseIntakeConstants.RIGHT_EXTENSION_CONSTANTS,
            V3_HorseIntakeConstants.RIGHT_EXTENSION_STATES.get(extensionState));

    leftManualPositionGoal =
        new Setpoint<>(
            Meters.of(V3_HorseIntakeConstants.EXTENSION_STOW_POSITION),
            Meters.of(0.01),
            Meters.of(V3_HorseIntakeConstants.MIN_EXTENSION),
            Meters.of(V3_HorseIntakeConstants.MAX_EXTENSION));
    rightManualPositionGoal =
        new Setpoint<>(
            Meters.of(V3_HorseIntakeConstants.EXTENSION_STOW_POSITION),
            Meters.of(0.01),
            Meters.of(V3_HorseIntakeConstants.MIN_EXTENSION),
            Meters.of(V3_HorseIntakeConstants.MAX_EXTENSION));

    extensionPosition = Meters.zero();

    manualReachedStow = false;
    previousExtensionState = extensionState;
    leftCompliant = false;
    rightCompliant = false;
    leftGainSlot = GainSlot.ZERO;
    rightGainSlot = GainSlot.ZERO;
  }

  @Trace
  @Override
  public void periodic() {
    // A characterization drives the extensions itself
    if (!characterizing) {
      applyExtensionGoals();
    }
    intakeRoller.setVoltageGoal(V3_HorseIntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    kickerRoller.setVoltageGoal(V3_HorseIntakeConstants.KICKER_ROLLER_STATES.get(rollerState));
    intakeRoller.periodic();
    kickerRoller.periodic();
    leftExtension.periodic();
    rightExtension.periodic();

    extensionPosition = Meters.of(leftExtension.getPosition().in(Meters));

    Logger.recordOutput("Intake/Rollers/State", rollerState.toString());
    Logger.recordOutput("Intake/Extension/State", extensionState.toString());
    Logger.recordOutput("Intake/Extension/Trigger", triggerSupplier.getAsDouble());
  }

  private void applyExtensionGoals() {
    boolean triggerAtStow =
        linearExtensionMap().in(Meters)
            <= V3_HorseIntakeConstants.EXTENSION_STOW_POSITION
                + V3_HorseIntakeConstants.AGITATE_AUTO_STOW_THRESHOLD;
    // Remembers whether this press reached the stow point, so release can stow even if the
    // intake hasn't physically gotten there yet.
    manualReachedStow =
        extensionState == ExtensionState.MANUAL_EXTEND && (manualReachedStow || triggerAtStow);

    if (extensionState == ExtensionState.MANUAL_EXTEND
        && triggerAtStow
        && isNearStow(V3_HorseIntakeConstants.AGITATE_AUTO_STOW_THRESHOLD)) {
      // Pulled in to stow: latch so releasing the trigger doesn't drive it back out.
      // The goal
      // check keeps a light press from an already-stowed intake from latching
      // immediately.
      extensionState = ExtensionState.STOW;
    }

    switch (extensionState) {
      case INTAKE, STOW:
        leftExtension.setPositionGoal(
            V3_HorseIntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
        rightExtension.setPositionGoal(
            V3_HorseIntakeConstants.RIGHT_EXTENSION_STATES.get(extensionState));
        break;
      case MANUAL_EXTEND:
        leftManualPositionGoal.setSetpoint(linearExtensionMap());
        rightManualPositionGoal.setSetpoint(linearExtensionMap());
        leftExtension.setPositionGoal(leftManualPositionGoal);
        rightExtension.setPositionGoal(rightManualPositionGoal);
        break;
      case ZERO, STOP, SLOW_MOVE, SQUISH, SYSID:
        // Open loop; the command that entered the state owns the extension output.
        break;
    }
    updateGainSlots();
  }

  private Distance linearExtensionMap() {
    // Blend linear with smoothstep so the ends of the trigger are finer than the
    // middle.
    // Slope is (1 - k) at the ends and (1 + k/2) in the middle.
    double t = Math.clamp(triggerSupplier.getAsDouble(), 0.0, 1.0);
    double k = V3_HorseIntakeConstants.TRIGGER_CURVE;
    double shaped = (1 - k) * t + k * t * t * (3 - 2 * t);
    Distance position =
        Meters.of(
            (1 - shaped)
                    * (V3_HorseIntakeConstants.EXTENSION_INTAKE_POSITION
                        - V3_HorseIntakeConstants.EXTENSION_STOW_POSITION)
                + V3_HorseIntakeConstants.EXTENSION_STOW_POSITION);
    return position;
  }

  // Firm (ZERO) while driving to a new goal, then compliant (ONE) until the state changes, so
  // getting pushed off the goal doesn't make it fight back.
  private void updateGainSlots() {
    boolean holdingGoal = extensionState == ExtensionState.INTAKE;
    if (!holdingGoal || extensionState != previousExtensionState) {
      leftCompliant = false;
      rightCompliant = false;
    } else {
      leftCompliant |= leftExtension.atPositionGoal();
      rightCompliant |= rightExtension.atPositionGoal();
    }
    previousExtensionState = extensionState;

    // setGainSlot sends a position request, so only touch it in closed loop states. STOW and
    // MANUAL_EXTEND still need it so a compliant slot left over from INTAKE gets cleared.
    if (holdingGoal
        || extensionState == ExtensionState.STOW
        || extensionState == ExtensionState.MANUAL_EXTEND) {
      GainSlot leftSlot = leftCompliant ? GainSlot.ONE : GainSlot.ZERO;
      GainSlot rightSlot = rightCompliant ? GainSlot.ONE : GainSlot.ZERO;
      if (leftSlot != leftGainSlot) {
        leftExtension.setPositionGainSlot(leftSlot);
        leftGainSlot = leftSlot;
      }
      if (rightSlot != rightGainSlot) {
        rightExtension.setPositionGainSlot(rightSlot);
        rightGainSlot = rightSlot;
      }
    }

    Logger.recordOutput("Intake/Extension/Left Compliant", leftCompliant);
    Logger.recordOutput("Intake/Extension/Right Compliant", rightCompliant);
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

  /**
   * Releases manual control, unless it already latched to stow. Stows only if the trigger reached
   * the stow point during this press, otherwise goes back out to intake.
   */
  public Command releaseManualExtend() {
    return Commands.runOnce(
        () -> {
          if (extensionState == ExtensionState.MANUAL_EXTEND) {
            extensionState = manualReachedStow ? ExtensionState.STOW : ExtensionState.INTAKE;
          }
        });
  }

  public Command zeroExtensions() {
    return Commands.sequence(
            Commands.runOnce(() -> rollerState = RollerState.STOP),
            Commands.runOnce(() -> extensionState = ExtensionState.ZERO),
            Commands.parallel(leftExtension.home(), rightExtension.home()),
            Commands.runOnce(() -> extensionState = ExtensionState.STOW))
        .finallyDo(
            interrupted -> {
              // Not zeroed, so don't trust position control; stop driving into the hard stop.
              if (interrupted) {
                extensionState = ExtensionState.STOP;
                leftExtension.setVoltageGoal(Volts.zero());
                rightExtension.setVoltageGoal(Volts.zero());
              }
            });
  }

  public Command resetIntakeZeroPosition() {
    return Commands.runOnce(
            () -> {
              leftExtension.setPosition(Meters.of(V3_HorseIntakeConstants.MIN_EXTENSION));
              rightExtension.setPosition(Meters.of(V3_HorseIntakeConstants.MIN_EXTENSION));
              extensionState = ExtensionState.STOW;
            })
        .ignoringDisable(true);
  }

  public Command slowMoveIn() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.SLOW_MOVE;
          leftExtension.setVoltageGoal(
              V3_HorseIntakeConstants.EXTENSION_SLOW_MOVE_VOLTAGE.times(-1));
          rightExtension.setVoltageGoal(
              V3_HorseIntakeConstants.EXTENSION_SLOW_MOVE_VOLTAGE.times(-1));
        });
  }

  public Command slowMoveOut() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.SLOW_MOVE;
          leftExtension.setVoltageGoal(V3_HorseIntakeConstants.EXTENSION_SLOW_MOVE_VOLTAGE);
          rightExtension.setVoltageGoal(V3_HorseIntakeConstants.EXTENSION_SLOW_MOVE_VOLTAGE);
        });
  }

  public Command stopExtensions() {
    return Commands.runOnce(
        () -> {
          extensionState = ExtensionState.STOP;
          leftExtension.setVoltageGoal(Volts.zero());
          rightExtension.setVoltageGoal(Volts.zero());
        });
  }

  public Command squishIntake() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (extensionState == ExtensionState.MANUAL_EXTEND
                      || extensionState == ExtensionState.STOW) {
                    return;
                  }
                  extensionState = ExtensionState.SQUISH;
                  leftExtension.setVoltageGoal(Volts.of(-1));
                  rightExtension.setVoltageGoal(Volts.of(-1));
                }),
            Commands.waitUntil(() -> extensionState != ExtensionState.SQUISH || isNearStow(0.01))
                .withTimeout(5))
        .finallyDo(
            () -> {
              if (extensionState == ExtensionState.SQUISH) {
                leftExtension.setVoltageGoal(Volts.zero());
                rightExtension.setVoltageGoal(Volts.zero());
                extensionState = ExtensionState.STOW;
              }
            });
  }

  private boolean isNearStow(double tolerance) {
    double threshold = V3_HorseIntakeConstants.EXTENSION_STOW_POSITION + tolerance;
    return leftExtension.getPosition().in(Meters) <= threshold
        && rightExtension.getPosition().in(Meters) <= threshold;
  }

  /**
   * Characterizes both extensions together in their config's unit and prints each one's plant and
   * gains.
   */
  public Command characterizeExtensions() {
    return characterizing(
        extensionRoutine(leftExtension.getConstants().characterization).command(),
        "Characterize Extensions");
  }

  private <U extends Unit> CharacterizationRoutine<U> extensionRoutine(
      CharacterizationConfig<U> config) {
    return new CharacterizationRoutine<>(
        config,
        this,
        List.of(
            leftExtension.characterizationTarget(config),
            rightExtension.characterizationTarget(config)));
  }

  /** Runs a characterization while periodic leaves the extensions' goals alone. */
  private Command characterizing(Command characterization, String name) {
    return Commands.runOnce(() -> characterizing = true)
        .andThen(characterization)
        .finallyDo(() -> characterizing = false)
        .withName(name);
  }
}
