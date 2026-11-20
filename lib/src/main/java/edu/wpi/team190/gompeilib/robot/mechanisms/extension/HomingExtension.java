package edu.wpi.team190.gompeilib.robot.mechanisms.extension;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.common.homing.CurrentHoming;
import edu.wpi.team190.gompeilib.robot.common.homing.Homeable;
import java.util.Objects;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.VoltageUnit;

/** An {@link Extension} that finds its position by current homing against a hard stop. */
public class HomingExtension extends Extension implements Homeable {
  private final CurrentHoming<DistanceUnit> homing;

  /**
   * Creates a homing extension.
   *
   * @param motorFactory how to build the extension's motors
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param name the extension's name; it logs under {@code <subsystem>/<name>}
   * @param constants the extension's constants
   * @param homing how the extension homes
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public HomingExtension(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ExtensionConstants constants,
      CurrentHoming.Config<DistanceUnit> homing,
      Setpoint<DistanceUnit> positionGoal,
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
   * Creates a homing extension with a position goal at zero, limited to its travel, and a 0 V goal.
   *
   * @param motorFactory how to build the extension's motors
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param name the extension's name; it logs under {@code <subsystem>/<name>}
   * @param constants the extension's constants
   * @param homing how the extension homes
   */
  public HomingExtension(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ExtensionConstants constants,
      CurrentHoming.Config<DistanceUnit> homing) {
    this(motorFactory, subsystem, name, constants, homing, defaultPositionGoal(constants));
  }

  /**
   * Creates a homing extension with a 0 V goal.
   *
   * @param motorFactory how to build the extension's motors
   * @param subsystem the subsystem that owns the extension, for its log prefix
   * @param name the extension's name; it logs under {@code <subsystem>/<name>}
   * @param constants the extension's constants
   * @param homing how the extension homes
   * @param positionGoal the initial position goal
   */
  public HomingExtension(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      String name,
      ExtensionConstants constants,
      CurrentHoming.Config<DistanceUnit> homing,
      Setpoint<DistanceUnit> positionGoal) {
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
  public CurrentHoming<DistanceUnit> homing() {
    return homing;
  }

  @Override
  @Trace
  public void periodic() {
    core().periodic();
  }
}
