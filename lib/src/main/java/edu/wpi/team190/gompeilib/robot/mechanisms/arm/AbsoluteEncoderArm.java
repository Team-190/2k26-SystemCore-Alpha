package edu.wpi.team190.gompeilib.robot.mechanisms.arm;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.common.encoder.EncoderSeed;
import edu.wpi.team190.gompeilib.robot.common.encoder.EncoderSeedable;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;

/**
 * An {@link Arm} with an absolute encoder on its output, which is read and logged under {@code
 * <topic>/Encoder} every loop and can seed the motors' position.
 */
public class AbsoluteEncoderArm extends Arm implements EncoderSeedable {
  private final EncoderSeed encoderSeed;

  /**
   * Creates an arm with an absolute encoder.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   * @param encoder the absolute encoder on the arm's output
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public AbsoluteEncoderArm(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ArmConstants constants,
      AbsoluteEncoderIO encoder,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    super(motorFactory, subsystem, name, constants, positionGoal, voltageGoal);
    encoderSeed = new EncoderSeed(core(), encoder);
  }

  /**
   * Creates an arm with an absolute encoder, a position goal at zero limited to its travel, and a 0
   * V goal.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   * @param encoder the absolute encoder on the arm's output
   */
  public AbsoluteEncoderArm(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ArmConstants constants,
      AbsoluteEncoderIO encoder) {
    this(motorFactory, subsystem, name, constants, encoder, defaultPositionGoal(constants));
  }

  /**
   * Creates an arm with an absolute encoder and a 0 V goal.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param name the arm's name; it logs under {@code <subsystem>/<name>}
   * @param constants the arm's constants
   * @param encoder the absolute encoder on the arm's output
   * @param positionGoal the initial position goal
   */
  public AbsoluteEncoderArm(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ArmConstants constants,
      AbsoluteEncoderIO encoder,
      Setpoint<AngleUnit> positionGoal) {
    this(
        motorFactory,
        subsystem,
        name,
        constants,
        encoder,
        positionGoal,
        defaultVoltageGoal(constants));
  }

  @Override
  public EncoderSeed encoderSeed() {
    return encoderSeed;
  }

  @Override
  @Trace
  public void periodic() {
    core().periodic();
  }
}
