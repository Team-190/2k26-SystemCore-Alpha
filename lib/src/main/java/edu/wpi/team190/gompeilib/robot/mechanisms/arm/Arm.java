package edu.wpi.team190.gompeilib.robot.mechanisms.arm;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.io.physics.PlantMechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismConversion;
import edu.wpi.team190.gompeilib.robot.common.core.PositionalMechanism;
import edu.wpi.team190.gompeilib.robot.common.core.PositionalMechanism.Travel;
import java.util.Optional;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;

/**
 * A position-controlled arm: one leader motor and any number of followers. It starts idle. An arm
 * that homes against a hard stop is a {@link HomingArm}; one with an absolute encoder on its output
 * is an {@link AbsoluteEncoderArm}.
 */
public class Arm
    extends PositionalMechanism<
        AngleUnit, AngularVelocityUnit, AngularAccelerationUnit, AngularPositionConstraints> {
  private static final MechanismConversion<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit>
      CONVERSION = MechanismConversion.angular();

  /**
   * Creates an arm.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public Arm(
      MotorIOFactory motorFactory,
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
        Optional.of(
            new Travel<>(
                constants.armParameters.minAngle(),
                constants.armParameters.maxAngle(),
                constants.constraints.characterizationMargin())),
        constants.closedLoopOutputType,
        positionGoal,
        voltageGoal,
        constants.currentOffsetStep,
        constants.dutyCycleOffsetStep,
        constants.characterization);
  }

  /**
   * Creates an arm with a position goal at zero limited to its travel and a 0 V goal.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   */
  public Arm(
      MotorIOFactory motorFactory, Subsystem subsystem, String name, ArmConstants constants) {
    this(motorFactory, subsystem, name, constants, defaultPositionGoal(constants));
  }

  /**
   * Creates an arm with a 0 V goal.
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
    this(motorFactory, subsystem, name, constants, positionGoal, defaultVoltageGoal(constants));
  }

  /**
   * Returns a position goal at zero, limited to the arm's travel.
   *
   * @param constants the arm's constants
   * @return the goal
   */
  protected static Setpoint<AngleUnit> defaultPositionGoal(ArmConstants constants) {
    return new Setpoint<>(
        Radians.of(0),
        constants.positionOffsetStep,
        constants.armParameters.minAngle(),
        constants.armParameters.maxAngle());
  }

  /**
   * Returns a 0 V goal, limited to ±12 V.
   *
   * @param constants the arm's constants
   * @return the goal
   */
  protected static Setpoint<VoltageUnit> defaultVoltageGoal(ArmConstants constants) {
    return new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12));
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
