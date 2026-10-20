package first.robot.v3_Horse_CV2_TR.subsystems.intake;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.mechanisms.extension.Extension;
import edu.wpi.team190.gompeilib.robot.mechanisms.roller.GenericRoller;
import edu.wpi.team190.gompeilib.sysid.CustomSysIdRoutine;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants.ExtensionState;
import first.robot.v3_Horse_CV2_TR.subsystems.intake.V3_Horse_CV2_TR_IntakeConstants.RollerState;
import java.util.List;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.measure.Distance;

public class V3_Horse_CV2_TR_Intake extends SubsystemBase {

  @Getter private ExtensionState extensionState;
  @Getter private RollerState rollerState;

  private final GenericRoller intakeRoller;

  public final Extension leftExtension;

  public final Extension rightExtension;

  private final GenericRoller kickerRoller;

  private DoubleSupplier triggerSupplier;

  private final Setpoint<DistanceUnit> leftManualPositionGoal;

  private final Setpoint<DistanceUnit> rightManualPositionGoal;

  @Getter private Distance extensionPosition;

  public V3_Horse_CV2_TR_Intake(
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
            V3_Horse_CV2_TR_IntakeConstants.INTAKE_ROLLER_CONSTANTS,
            "IntakeRoller",
            V3_Horse_CV2_TR_IntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));

    kickerRoller =
        new GenericRoller(
            kickerRollerMotors,
            this,
            V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_CONSTANTS,
            "KickerRoller",
            V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_STATES.get(rollerState));

    leftExtension =
        new Extension(
            leftExtensionMotors,
            this,
            "LeftExtension",
            V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_CONSTANTS,
            V3_Horse_CV2_TR_IntakeConstants.LEFT_EXTENSION_STATES.get(extensionState));
    rightExtension =
        new Extension(
            rightExtensionMotors,
            this,
            "RightExtension",
            V3_Horse_CV2_TR_IntakeConstants.RIGHT_EXTENSION_CONSTANTS,
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

      case MANUAL_RELEASE:
        break;
    }
    // if (extensionState != ExtensionState.OVERRIDE) {
    // updateGainSlots();
    // }
    intakeRoller.setVoltageGoal(
        V3_Horse_CV2_TR_IntakeConstants.INTAKE_ROLLER_STATES.get(rollerState));
    kickerRoller.setVoltageGoal(
        V3_Horse_CV2_TR_IntakeConstants.KICKER_ROLLER_STATES.get(rollerState));
    intakeRoller.periodic();
    kickerRoller.periodic();
    leftExtension.periodic();
    rightExtension.periodic();

    extensionPosition = Meters.of(leftExtension.getPosition().in(Meters));

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
  // private void updateGainSlots() {
  //   leftExtension.setGainSlot(leftExtension.atPositionGoal() ? GainSlot.ZERO : GainSlot.ONE);
  //   rightExtension.setGainSlot(rightExtension.atPositionGoal() ? GainSlot.ZERO : GainSlot.ONE);
  // }

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
        Commands.parallel(leftExtension.home(), rightExtension.home()),
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

  public Command squishIntake() {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  if (extensionState == ExtensionState.MANUAL_EXTEND
                      || extensionState == ExtensionState.MANUAL_RELEASE
                      || extensionState == ExtensionState.STOW) {
                    return;
                  }
                  extensionState = ExtensionState.OVERRIDE;
                  leftExtension.setVoltageGoal(Volts.of(-2.5));
                  rightExtension.setVoltageGoal(Volts.of(-2.5));
                }),
            Commands.waitUntil(() -> extensionState != ExtensionState.OVERRIDE || isNearStow(0.01))
                .withTimeout(5))
        .finallyDo(
            () -> {
              if (extensionState == ExtensionState.OVERRIDE) {
                leftExtension.setVoltageGoal(Volts.zero());
                rightExtension.setVoltageGoal(Volts.zero());
                extensionState = ExtensionState.STOW;
              }
            });
  }

  private boolean isNearStow(double tolerance) {
    double threshold = V3_Horse_CV2_TR_IntakeConstants.EXTENSION_STOW_POSITION + tolerance;
    return leftExtension.getPosition().in(Meters) <= threshold
        && rightExtension.getPosition().in(Meters) <= threshold;
  }

  public Command sysID() {
    return setExtensionState(ExtensionState.OVERRIDE)
        .andThen(
            new CustomSysIdRoutine<>(
                    leftExtension.getConstants().sysId,
                    this,
                    List.of(
                        leftExtension.voltageSysIdTarget(), rightExtension.voltageSysIdTarget()))
                .command());
  }
}
