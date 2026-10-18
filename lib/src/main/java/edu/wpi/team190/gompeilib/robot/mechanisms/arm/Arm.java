package edu.wpi.team190.gompeilib.robot.mechanisms.arm;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.physics.ArmMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.mechanisms.ConstraintConfigurable;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.EncoderSeed;
import edu.wpi.team190.gompeilib.robot.mechanisms.EncoderSeedable;
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
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;

/**
 * A position-controlled arm: one leader motor, any number of followers, an optional absolute
 * encoder on the arm's output that can seed the motors' position, and optional current homing
 * against a hard stop. It starts idle.
 */
public class Arm
    implements PositionControllable<AngleUnit>,
        VoltageControllable,
        MotionReadable<AngleUnit, AngularVelocityUnit>,
        ConstraintConfigurable<AngularPositionConstraints>,
        Homeable,
        EncoderSeedable {
  private static final MechanismConversion<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit>
      CONVERSION = MechanismConversion.angular();

  private final MechanismCore core;
  private final PositionControl<AngleUnit> positionControl;
  private final VoltageControl voltageControl;
  private final EncoderSeed encoderSeed;
  private final Optional<CurrentHoming<AngleUnit>> homing;

  private final SysIdRoutine characterizationRoutine;

  /**
   * Creates an arm.
   *
   * @param motorFactory how to build the arm's motors
   * @param encoder the absolute encoder on the arm's output, or empty for none
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public Arm(
      MotorIOFactory motorFactory,
      Optional<AbsoluteEncoderIO> encoder,
      Subsystem subsystem,
      String name,
      ArmConstants constants,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    MotorControllerGroup motors =
        new MotorControllerGroup(
            constants.motor.toBuilder().withMotionProfile(profile(constants.constraints)).build(),
            motorFactory);
    core = new MechanismCore(motors, subsystem, name);
    boolean foc = constants.motor.enableFOC;
    positionControl =
        new PositionControl<>(
            core, CONVERSION, positionGoal, () -> constants.constraints.goalTolerance().get(), foc);
    voltageControl = new VoltageControl(core, voltageGoal, foc);
    encoderSeed = new EncoderSeed(core, encoder);
    homing =
        constants.homing.map(config -> new CurrentHoming<>(core, positionControl, config, foc));

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(9),
                Seconds.of(12),
                (state) -> Logger.recordOutput(core.getTopic() + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                voltage ->
                    core.setControl(
                        new OpenLoopRequest.VoltageOutput(voltage, foc, RequestOptions.DEFAULT)),
                null,
                subsystem));

    TuningBindings.bindGains(motors, constants.motor);
    TuningBindings.bindConstraints(motors, constants.constraints, Arm::profile);
  }

  /**
   * Creates an arm without an absolute encoder, with a position goal at zero limited to its travel
   * and a 0 V goal.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   */
  public Arm(
      MotorIOFactory motorFactory, Subsystem subsystem, String name, ArmConstants constants) {
    this(
        motorFactory,
        subsystem,
        name,
        constants,
        new Setpoint<>(
            Radians.of(0),
            constants.positionOffsetStep,
            constants.armParameters.minAngle(),
            constants.armParameters.maxAngle()));
  }

  /**
   * Creates an arm without an absolute encoder, with a 0 V goal.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   * @param positionGoal the initial position goal
   */
  public Arm(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ArmConstants constants,
      Setpoint<AngleUnit> positionGoal) {
    this(
        motorFactory,
        Optional.empty(),
        subsystem,
        name,
        constants,
        positionGoal,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  /**
   * Builds the physics model for simulating an arm. It starts at its lowest angle.
   *
   * @param constants the arm's constants
   * @return the physics model
   */
  public static MechanismSim simMechanism(ArmConstants constants) {
    ArmConstants.ArmParameters parameters = constants.armParameters;
    return new ArmMechanismSim(
        parameters.armGearbox(),
        constants.motor.gearRatio,
        parameters.momentOfInertia(),
        parameters.length(),
        parameters.minAngle(),
        parameters.maxAngle(),
        parameters.simulateGravity(),
        parameters.minAngle());
  }

  private static MotionProfileConfig profile(AngularPositionConstraints constraints) {
    return CONVERSION.trapezoidal(
        constraints.maxVelocity().get(), constraints.maxAcceleration().get());
  }

  @Override
  public MechanismCore core() {
    return core;
  }

  @Override
  public PositionControl<AngleUnit> positionControl() {
    return positionControl;
  }

  @Override
  public VoltageControl voltageControl() {
    return voltageControl;
  }

  @Override
  public EncoderSeed encoderSeed() {
    return encoderSeed;
  }

  @Override
  public CurrentHoming<AngleUnit> homing() {
    return homing.orElseThrow(
        () -> new IllegalStateException(core.getTopic() + " has no homing config"));
  }

  @Override
  public MechanismConversion<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit> conversion() {
    return CONVERSION;
  }

  @Override
  public MotionProfileConfig motionProfile(AngularPositionConstraints constraints) {
    return profile(constraints);
  }

  @Override
  @Trace
  public void periodic() {
    core.periodic();
  }

  public Command sysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(core::idle),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.FORWARD),
        Commands.waitSeconds(1.0),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.REVERSE),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.FORWARD),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.REVERSE));
  }
}
