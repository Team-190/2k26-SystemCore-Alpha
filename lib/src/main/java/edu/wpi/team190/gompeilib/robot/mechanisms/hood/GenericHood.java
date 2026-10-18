package edu.wpi.team190.gompeilib.robot.mechanisms.hood;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.Arm;
import edu.wpi.team190.gompeilib.robot.mechanisms.arm.ArmConstants;
import java.util.Optional;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;

/**
 * A hood: a short-range {@link Arm} that homes against a hard stop. Its constants must include
 * {@link ArmConstants#homing}; everything else is the arm's.
 */
public class GenericHood extends Arm {
  /**
   * Creates a hood.
   *
   * @param motorFactory how to build the hood's motors
   * @param subsystem the subsystem that owns the hood, for its log prefix
   * @param name the hood's name; it logs under {@code <subsystem>/<name>}
   * @param constants the hood's constants
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   * @throws IllegalArgumentException if the constants have no homing config
   */
  public GenericHood(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ArmConstants constants,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    super(
        motorFactory,
        Optional.empty(),
        subsystem,
        name,
        requireHoming(constants),
        positionGoal,
        voltageGoal);
  }

  private static ArmConstants requireHoming(ArmConstants constants) {
    if (constants.homing.isEmpty()) {
      throw new IllegalArgumentException("A hood needs a homing config");
    }
    return constants;
  }
}
