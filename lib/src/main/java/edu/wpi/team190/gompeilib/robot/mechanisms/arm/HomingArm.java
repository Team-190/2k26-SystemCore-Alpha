package edu.wpi.team190.gompeilib.robot.mechanisms.arm;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.common.homing.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.common.homing.Homeable;
import java.util.Objects;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;

/** An {@link Arm} that finds its position by current homing against a hard stop. */
public class HomingArm extends Arm implements Homeable {
  private final CurrentHoming<AngleUnit> homing;

  /**
   * Creates a homing arm.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   * @param homing how the arm homes
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public HomingArm(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ArmConstants constants,
      CurrentHoming.Config<AngleUnit> homing,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    super(motorFactory, subsystem, name, constants, positionGoal, voltageGoal);
    this.homing =
        new CurrentHoming<>(
            core(),
            positionControl(),
            voltageControl(),
            currentControl(),
            Objects.requireNonNull(homing, "homing"));
  }

  /**
   * Creates a homing arm with a position goal at zero limited to its travel and a 0 V goal.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   * @param homing how the arm homes
   */
  public HomingArm(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ArmConstants constants,
      CurrentHoming.Config<AngleUnit> homing) {
    this(motorFactory, subsystem, name, constants, homing, defaultPositionGoal(constants));
  }

  /**
   * Creates a homing arm with a 0 V goal.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   * @param homing how the arm homes
   * @param positionGoal the initial position goal
   */
  public HomingArm(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ArmConstants constants,
      CurrentHoming.Config<AngleUnit> homing,
      Setpoint<AngleUnit> positionGoal) {
    this(
        motorFactory,
        subsystem,
        name,
        constants,
        homing,
        positionGoal,
        defaultVoltageGoal(constants));
  }

  @Override
  public CurrentHoming<AngleUnit> homing() {
    return homing;
  }

  @Override
  @Trace
  public void periodic() {
    core().periodic();
  }
}
