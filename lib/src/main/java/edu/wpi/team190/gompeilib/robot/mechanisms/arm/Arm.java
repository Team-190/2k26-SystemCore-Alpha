package edu.wpi.team190.gompeilib.robot.mechanisms.arm;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.io.physics.PlantMechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismConversion;
import edu.wpi.team190.gompeilib.robot.common.core.PositionalMechanism;
import edu.wpi.team190.gompeilib.robot.common.encoder.EncoderSeed;
import edu.wpi.team190.gompeilib.robot.common.encoder.EncoderSeedable;
import java.util.Optional;
import org.wpilib.command2.Subsystem;
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
    extends PositionalMechanism<
        AngleUnit, AngularVelocityUnit, AngularAccelerationUnit, AngularPositionConstraints>
    implements EncoderSeedable {
  private static final MechanismConversion<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit>
      CONVERSION = MechanismConversion.angular();

  private final EncoderSeed encoderSeed;

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
    super(
        motorFactory,
        constants.motor.toBuilder().withMotionProfile(profile(constants.constraints)).build(),
        subsystem,
        name,
        CONVERSION,
        constants.constraints,
        () -> constants.constraints.goalTolerance().get(),
        constants.armParameters.minAngle(),
        constants.armParameters.maxAngle(),
        constants.constraints.characterizationMargin(),
        constants.closedLoopOutputType,
        positionGoal,
        voltageGoal,
        constants.currentOffsetStep,
        constants.dutyCycleOffsetStep,
        constants.homing,
        constants.characterization);
    encoderSeed = new EncoderSeed(core(), encoder);
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
   * Builds the physics model for simulating an arm: its plant, between hard stops at its travel
   * limits. It starts at its lowest angle.
   *
   * @param constants the arm's constants
   * @return the physics model
   */
  public static MechanismSim simMechanism(ArmConstants constants) {
    ArmConstants.ArmParameters parameters = constants.armParameters;
    return new PlantMechanismSim(
        parameters.armGearbox(),
        constants.motor.gearRatio,
        parameters.plant(),
        parameters.minAngle(),
        parameters.maxAngle(),
        parameters.minAngle());
  }

  private static MotionProfileConfig profile(AngularPositionConstraints constraints) {
    return CONVERSION.trapezoidal(
        constraints.maxVelocity().get(), constraints.maxAcceleration().get());
  }

  @Override
  public EncoderSeed encoderSeed() {
    return encoderSeed;
  }

  @Override
  public MotionProfileConfig motionProfile(AngularPositionConstraints constraints) {
    return profile(constraints);
  }

  @Override
  @Trace
  public void periodic() {
    core().periodic();
  }

  @Override
  public Optional<GravityType> characterizationGravity() {
    return Optional.of(GravityType.COSINE);
  }
}
