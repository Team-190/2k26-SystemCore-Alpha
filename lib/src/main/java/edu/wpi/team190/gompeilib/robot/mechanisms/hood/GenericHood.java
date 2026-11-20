package edu.wpi.team190.gompeilib.robot.mechanisms.hood;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.common.homing.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.ArmConstants;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.HomingArm;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;

/** A hood: a short-range {@link HomingArm}; everything else is the arm's. */
public class GenericHood extends HomingArm {
  /**
   * Creates a hood.
   *
   * @param motorFactory how to build the hood's motors
   * @param subsystem the subsystem that owns the hood, for its log prefix
   * @param name the hood's name; it logs under {@code <subsystem>/<name>}
   * @param constants the hood's constants
   * @param homing how the hood homes
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public GenericHood(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ArmConstants constants,
      CurrentHoming.Config<AngleUnit> homing,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    super(motorFactory, subsystem, name, constants, homing, positionGoal, voltageGoal);
  }

  @Override
  @Trace
  public void periodic() {
    core().periodic();
  }
}
