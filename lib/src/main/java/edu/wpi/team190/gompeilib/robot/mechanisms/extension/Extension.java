package edu.wpi.team190.gompeilib.robot.mechanisms.extension;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SoftLimits;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.physics.ElevatorMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.mechanisms.ConstraintConfigurable;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.Homeable;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismConversion;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import edu.wpi.team190.gompeilib.robot.mechanisms.MotionReadable;
import edu.wpi.team190.gompeilib.robot.mechanisms.PositionControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.PositionControllable;
import edu.wpi.team190.gompeilib.robot.mechanisms.TuningBindings;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControllable;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Voltage;

/**
 * A linear extension driven through a drum: one leader motor and any number of followers. The
 * motors work in drum rotations; the extension's drum conversion turns travel into rotation and
 * back. It starts idle.
 */
public class Extension
    implements PositionControllable<DistanceUnit>,
        VoltageControllable,
        MotionReadable<DistanceUnit, LinearVelocityUnit>,
        ConstraintConfigurable<LinearConstraints>,
        Homeable {
  @Getter private final ExtensionConstants constants;
  private final MechanismConversion<DistanceUnit, LinearVelocityUnit, LinearAccelerationUnit>
      conversion;
  private final MechanismCore core;
  private final PositionControl<DistanceUnit> positionControl;
  private final VoltageControl voltageControl;
  private final Optional<CurrentHoming<DistanceUnit>> homing;

  private final SysIdRoutine characterizationRoutine;

  /**
   * Creates an extension.
   *
   * @param motorFactory how to build the extension's motors
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param name the extension's name; it logs under {@code <subsystem>/<name>}
   * @param constants the extension's constants
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public Extension(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ExtensionConstants constants,
      Setpoint<DistanceUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    this.constants = constants;
    conversion = MechanismConversion.drum(constants.drumRadius);
    ExtensionConstants.ExtensionParameters parameters = constants.extensionParameters;
    MotorControllerGroup motors =
        new MotorControllerGroup(
            constants.motor.toBuilder()
                .withSoftLimits(
                    SoftLimits.between(
                        conversion.toMotorPosition(parameters.minLength()),
                        conversion.toMotorPosition(parameters.maxLength())))
                .withMotionProfile(motionProfile(constants.constraints))
                .build(),
            motorFactory);
    core = new MechanismCore(motors, subsystem, name);
    boolean foc = constants.motor.enableFOC;
    positionControl =
        new PositionControl<>(
            core, conversion, positionGoal, () -> constants.constraints.goalTolerance().get(), foc);
    voltageControl = new VoltageControl(core, voltageGoal, foc);
    homing =
        constants.homing.map(config -> new CurrentHoming<>(core, positionControl, config, foc));

    characterizationRoutine = createCharacterizationRoutine(subsystem, this);

    TuningBindings.bindGains(motors, constants.motor);
    TuningBindings.bindConstraints(motors, constants.constraints, this::motionProfile);
  }

  /**
   * Creates an extension with a position goal at zero, limited to its travel, and a 0 V goal.
   *
   * @param motorFactory how to build the extension's motors
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param name the extension's name; it logs under {@code <subsystem>/<name>}
   * @param constants the extension's constants
   */
  public Extension(
      MotorIOFactory motorFactory, Subsystem subsystem, String name, ExtensionConstants constants) {
    this(
        motorFactory,
        subsystem,
        name,
        constants,
        new Setpoint<>(
            Meters.of(0),
            constants.heightOffsetStep,
            constants.extensionParameters.minLength(),
            constants.extensionParameters.maxLength()));
  }

  /**
   * Creates an extension with a 0 V goal.
   *
   * @param motorFactory how to build the extension's motors
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param name the extension's name; it logs under {@code <subsystem>/<name>}
   * @param constants the extension's constants
   * @param positionGoal the initial position goal
   */
  public Extension(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ExtensionConstants constants,
      Setpoint<DistanceUnit> positionGoal) {
    this(
        motorFactory,
        subsystem,
        name,
        constants,
        positionGoal,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  /**
   * Builds the physics model for simulating an extension. It starts at the shortest extension.
   *
   * @param constants the extension's constants
   * @return the physics model
   */
  public static MechanismSim simMechanism(ExtensionConstants constants) {
    ExtensionConstants.ExtensionParameters parameters = constants.extensionParameters;
    return new ElevatorMechanismSim(
        parameters.carriageGearbox(),
        constants.motor.gearRatio,
        parameters.carriageMass(),
        constants.drumRadius,
        parameters.minLength(),
        parameters.maxLength(),
        parameters.simulateGravity(),
        parameters.minLength());
  }

  @Override
  public MechanismCore core() {
    return core;
  }

  @Override
  public PositionControl<DistanceUnit> positionControl() {
    return positionControl;
  }

  @Override
  public VoltageControl voltageControl() {
    return voltageControl;
  }

  @Override
  public CurrentHoming<DistanceUnit> homing() {
    return homing.orElseThrow(
        () -> new IllegalStateException(core.getTopic() + " has no homing config"));
  }

  @Override
  public MechanismConversion<DistanceUnit, LinearVelocityUnit, LinearAccelerationUnit>
      conversion() {
    return conversion;
  }

  @Override
  public MotionProfileConfig motionProfile(LinearConstraints constraints) {
    return conversion.trapezoidal(
        constraints.maxVelocity().get(), constraints.maxAcceleration().get());
  }

  @Override
  @Trace
  public void periodic() {
    core.periodic();
  }

  private void setVoltage(Voltage voltage) {
    core.setControl(
        new OpenLoopRequest.VoltageOutput(
            voltage, constants.motor.enableFOC, RequestOptions.DEFAULT));
  }

  public Command runSysIdRoutine() {
    return runSysIdRoutine(characterizationRoutine, this);
  }

  /** Characterizes several extensions owned by the same subsystem in one pass. */
  public static Command runSysIdRoutine(Subsystem subsystem, Extension... extensions) {
    return runSysIdRoutine(createCharacterizationRoutine(subsystem, extensions), extensions);
  }

  private static SysIdRoutine createCharacterizationRoutine(
      Subsystem subsystem, Extension... extensions) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(1).per(Second),
            Volts.of(3),
            Seconds.of(3),
            (state) -> {
              for (Extension extension : extensions) {
                Logger.recordOutput(extension.core.getTopic() + "/SysIdState", state.toString());
              }
            }),
        new SysIdRoutine.Mechanism(
            (voltage) -> {
              for (Extension extension : extensions) {
                extension.setVoltage(voltage);
              }
            },
            null,
            subsystem));
  }

  /**
   * Returns true once any extension has reached the end stop in the direction of travel. The check
   * is one-sided so a fast dynamic test can't overshoot the tolerance band and keep driving.
   */
  private static boolean anyAtEndStop(SysIdRoutine.Direction direction, Extension... extensions) {
    for (Extension extension : extensions) {
      double position = extension.getPosition().in(Meters);
      double tolerance = extension.constants.constraints.goalTolerance().get().in(Meters);
      ExtensionConstants.ExtensionParameters parameters = extension.constants.extensionParameters;
      if (direction == SysIdRoutine.Direction.FORWARD
          ? position >= parameters.maxLength().in(Meters) - tolerance
          : position <= parameters.minLength().in(Meters) + tolerance) {
        return true;
      }
    }
    return false;
  }

  private static Command runSysIdRoutine(SysIdRoutine routine, Extension... extensions) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              for (Extension extension : extensions) {
                extension.idle();
              }
            }),
        routine
            .quasistatic(SysIdRoutine.Direction.FORWARD)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.FORWARD, extensions)),
        Commands.waitSeconds(1.0),
        routine
            .quasistatic(SysIdRoutine.Direction.REVERSE)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.REVERSE, extensions)),
        Commands.waitSeconds(1.0),
        routine
            .dynamic(SysIdRoutine.Direction.FORWARD)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.FORWARD, extensions)),
        Commands.waitSeconds(1.0),
        routine
            .dynamic(SysIdRoutine.Direction.REVERSE)
            .until(() -> anyAtEndStop(SysIdRoutine.Direction.REVERSE, extensions)));
  }
}
