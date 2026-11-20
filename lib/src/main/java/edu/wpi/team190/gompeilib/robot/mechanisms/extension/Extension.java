package edu.wpi.team190.gompeilib.robot.mechanisms.extension;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SoftLimits;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.io.physics.PlantMechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismConversion;
import edu.wpi.team190.gompeilib.robot.common.core.PositionalMechanism;
import edu.wpi.team190.gompeilib.robot.common.core.PositionalMechanism.Travel;
import java.util.Optional;
import lombok.Getter;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * A linear extension: one leader motor and any number of followers, driving a drum, sprocket, or
 * screw. The motors work in gearbox-output rotations; the extension's linear conversion turns
 * travel into rotation and back. It starts idle. An extension that homes against a hard stop is a
 * {@link HomingExtension}.
 */
public class Extension
    extends PositionalMechanism<
        DistanceUnit, LinearVelocityUnit, LinearAccelerationUnit, LinearConstraints> {
  @Getter private final ExtensionConstants constants;

  /**
   * Creates an extension.
   *
   * @param motorFactory how to build the extension's motors
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param name the extension's name; it logs under {@code <subsystem>/<name>}
   * @param constants the extension's constants
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   * @throws IllegalArgumentException if the motor config has slots; set {@link
   *     ExtensionConstants#slots} instead
   */
  public Extension(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ExtensionConstants constants,
      Setpoint<DistanceUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    super(
        motorFactory,
        motorConfig(constants, name),
        subsystem,
        name,
        MechanismConversion.linear(constants.distancePerRadian),
        constants.constraints,
        () -> constants.constraints.goalTolerance().get(),
        Optional.of(
            new Travel<>(
                constants.extensionParameters.minLength(),
                constants.extensionParameters.maxLength(),
                constants.constraints.characterizationMargin())),
        constants.closedLoopOutputType,
        positionGoal,
        voltageGoal,
        constants.currentOffsetStep,
        constants.dutyCycleOffsetStep,
        constants.characterization);
    this.constants = constants;
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
    this(motorFactory, subsystem, name, constants, defaultPositionGoal(constants));
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
    this(motorFactory, subsystem, name, constants, positionGoal, defaultVoltageGoal(constants));
  }

  /**
   * Returns a position goal at zero, limited to the extension's travel.
   *
   * @param constants the extension's constants
   * @return the goal
   */
  protected static Setpoint<DistanceUnit> defaultPositionGoal(ExtensionConstants constants) {
    return new Setpoint<>(
        Meters.of(0),
        constants.heightOffsetStep,
        constants.extensionParameters.minLength(),
        constants.extensionParameters.maxLength());
  }

  /**
   * Returns a 0 V goal, limited to ±12 V.
   *
   * @param constants the extension's constants
   * @return the goal
   */
  protected static Setpoint<VoltageUnit> defaultVoltageGoal(ExtensionConstants constants) {
    return new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12));
  }

  /**
   * Builds the leader's config: soft limits at the travel limits, the motion profile, and each
   * slot's gains converted from per-distance to per-radian.
   */
  private static MotorConfig motorConfig(ExtensionConstants constants, String name) {
    if (!constants.motor.slots.isEmpty()) {
      throw new IllegalArgumentException(
          name + "'s motor slots must be empty; set its gains per distance in slots instead");
    }
    MechanismConversion<DistanceUnit, LinearVelocityUnit, LinearAccelerationUnit> conversion =
        MechanismConversion.linear(constants.distancePerRadian);
    ExtensionConstants.ExtensionParameters parameters = constants.extensionParameters;
    MotorConfig.MotorConfigBuilder motorBuilder =
        constants.motor.toBuilder()
            .withSoftLimits(
                SoftLimits.between(
                    conversion.toMotorPosition(parameters.minLength()),
                    conversion.toMotorPosition(parameters.maxLength())))
            .withMotionProfile(profile(conversion, constants.constraints));
    constants.slots.forEach(
        (slot, gains) ->
            motorBuilder.withSlot(
                slot, SlotConfig.position(conversion.toMotorPositionGains(gains))));
    return motorBuilder.build();
  }

  private static MotionProfileConfig profile(
      MechanismConversion<DistanceUnit, LinearVelocityUnit, LinearAccelerationUnit> conversion,
      LinearConstraints constraints) {
    return conversion.trapezoidal(
        constraints.maxVelocity().get(), constraints.maxAcceleration().get());
  }

  /**
   * Builds the physics model for simulating an extension: its plant at the gearbox output, between
   * hard stops at its travel limits. It starts at the shortest extension.
   *
   * @param constants the extension's constants
   * @return the physics model
   */
  public static MechanismSim simMechanism(ExtensionConstants constants) {
    ExtensionConstants.ExtensionParameters parameters = constants.extensionParameters;
    double metersPerRadian = constants.distancePerRadian.in(Meters);
    Angle min = Radians.of(parameters.minLength().in(Meters) / metersPerRadian);
    return new PlantMechanismSim(
        parameters.carriageGearbox(),
        constants.motor.gearRatio,
        parameters.plant(),
        min,
        Radians.of(parameters.maxLength().in(Meters) / metersPerRadian),
        min);
  }

  @Override
  public MotionProfileConfig motionProfile(LinearConstraints constraints) {
    return profile(conversion(), constraints);
  }

  @Override
  @Trace
  public void periodic() {
    core().periodic();
  }

  @Override
  public Optional<GravityType> characterizationGravity() {
    return Optional.of(GravityType.CONSTANT);
  }

  @Override
  public Optional<Distance> characterizationDistancePerRadian() {
    return Optional.of(constants.distancePerRadian);
  }
}
