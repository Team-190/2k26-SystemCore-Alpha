package edu.wpi.team190.gompeilib.subsystems.hood;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.arm.Arm;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmConstants;
import java.util.Optional;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

/**
 * A hood: a short-range {@link Arm} that homes against a hard stop. Its constants must include
 * {@link ArmConstants#homing}.
 */
public class GenericHood {
  private final Arm arm;

  /**
   * Creates a hood.
   *
   * @param motorFactory how to build the hood's motors
   * @param subsystem the subsystem that owns the hood, for its log prefix
   * @param index the hood's index, appended to the log prefix
   * @param constants the hood's constants
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   * @throws IllegalArgumentException if the constants have no homing config
   */
  public GenericHood(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      int index,
      ArmConstants constants,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    if (constants.homing.isEmpty()) {
      throw new IllegalArgumentException("A hood needs a homing config");
    }
    arm =
        new Arm(
            motorFactory, Optional.empty(), subsystem, index, constants, positionGoal, voltageGoal);
  }

  /**
   * Creates a hood with a 0 V goal.
   *
   * @param motorFactory how to build the hood's motors
   * @param subsystem the subsystem that owns the hood, for its log prefix
   * @param index the hood's index, appended to the log prefix
   * @param constants the hood's constants
   * @param positionGoal the initial position goal
   */
  public GenericHood(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      int index,
      ArmConstants constants,
      Setpoint<AngleUnit> positionGoal) {
    this(
        motorFactory,
        subsystem,
        index,
        constants,
        positionGoal,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  /**
   * Creates a hood with a position goal at zero, limited to its travel, and a 0 V goal.
   *
   * @param motorFactory how to build the hood's motors
   * @param subsystem the subsystem that owns the hood, for its log prefix
   * @param index the hood's index, appended to the log prefix
   * @param constants the hood's constants
   */
  public GenericHood(
      MotorIOFactory motorFactory, Subsystem subsystem, int index, ArmConstants constants) {
    this(
        motorFactory,
        subsystem,
        index,
        constants,
        new Setpoint<>(
            Radians.zero(),
            constants.positionOffsetStep.getMeasure(),
            constants.armParameters.minAngle().getMeasure(),
            constants.armParameters.maxAngle().getMeasure()));
  }

  /**
   * Builds the physics model for simulating a hood. It starts at its lowest angle.
   *
   * @param constants the hood's constants
   * @return the physics model
   */
  public static MechanismSim simMechanism(ArmConstants constants) {
    return Arm.simMechanism(constants);
  }

  @Trace
  public void periodic() {
    arm.periodic();
  }

  public Setpoint<AngleUnit> getPositionGoal() {
    return arm.getPositionGoal();
  }

  public Setpoint<VoltageUnit> getVoltageGoal() {
    return arm.getVoltageGoal();
  }

  public void setVoltageGoal(Voltage voltageGoal) {
    arm.setVoltageGoal(voltageGoal);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltageGoal) {
    arm.setVoltageGoal(voltageGoal);
  }

  public void setPositionGoal(Rotation2d positionGoal) {
    arm.setPositionGoal(positionGoal);
  }

  public void setPositionGoal(Setpoint<AngleUnit> positionGoal) {
    arm.setPositionGoal(positionGoal);
  }

  public boolean atPositionGoal() {
    return arm.atPositionGoal();
  }

  public boolean atPositionGoal(Rotation2d position) {
    return arm.atPositionGoal(position);
  }

  public boolean atVoltageGoal() {
    return arm.atVoltageGoal();
  }

  public boolean atVoltageGoal(Voltage voltage) {
    return arm.atVoltageGoal(voltage);
  }

  /**
   * Waits a loop, so periodic applies the latest goal, then until the hood is at its goal.
   *
   * @return the command
   */
  public Command waitUntilAtGoal() {
    return Commands.waitSeconds(GompeiLib.getLoopPeriod()).andThen(arm.waitUntilAtGoal());
  }

  public void setGains(Gains gains) {
    arm.updateGains(gains, GainSlot.ZERO);
  }

  public void setProfile(AngularPositionConstraints constraints) {
    arm.updateConstraints(constraints);
  }

  /**
   * Drives the hood into its hard stop until the torque current reaches the homing threshold, then
   * sets its position to the stop's angle.
   *
   * @return the homing command
   */
  public Command resetHoodZero() {
    return arm.home();
  }

  public Command runSysIdRoutine() {
    return arm.sysIdRoutine();
  }

  public Rotation2d getAngle() {
    return arm.getArmPosition();
  }

  public AngularVelocity getVelocity() {
    return arm.getVelocity();
  }
}
