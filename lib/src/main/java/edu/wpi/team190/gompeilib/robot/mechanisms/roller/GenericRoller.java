package edu.wpi.team190.gompeilib.robot.mechanisms.roller;

import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.io.physics.PlantMechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import edu.wpi.team190.gompeilib.robot.common.openloop.CurrentControllable;
import edu.wpi.team190.gompeilib.robot.common.openloop.DutyCycleControllable;
import edu.wpi.team190.gompeilib.robot.common.openloop.OpenLoopControl;
import edu.wpi.team190.gompeilib.robot.common.openloop.OpenLoopControls;
import edu.wpi.team190.gompeilib.robot.common.openloop.VoltageControllable;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.VoltageUnit;

/**
 * A roller that can be driven in voltage, current, or duty cycle: one leader motor and any number
 * of followers. It starts driving its voltage goal, so it sends that goal every loop.
 */
public class GenericRoller
    implements VoltageControllable, CurrentControllable, DutyCycleControllable {
  private final MechanismCore core;
  private final OpenLoopControls openLoop;

  /**
   * Creates a roller.
   *
   * @param motorFactory how to build the roller's motors
   * @param subsystem the subsystem that owns the roller, for its log prefix
   * @param constants the roller's constants
   * @param name the roller's name; it logs under {@code <subsystem>/<name>}
   * @param voltageGoal the initial voltage goal
   */
  public GenericRoller(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      GenericRollerConstants constants,
      String name,
      Setpoint<VoltageUnit> voltageGoal) {
    core =
        new MechanismCore(new MotorControllerGroup(constants.motor, motorFactory), subsystem, name);
    openLoop =
        OpenLoopControls.create(
            core,
            constants.motor,
            voltageGoal,
            constants.currentOffsetStep,
            constants.dutyCycleOffsetStep);
    core.activate(openLoop.voltage());
  }

  /**
   * Creates a roller with a 0 V goal, limited to ±12 V.
   *
   * @param motorFactory how to build the roller's motors
   * @param subsystem the subsystem that owns the roller, for its log prefix
   * @param constants the roller's constants
   * @param name the roller's name; it logs under {@code <subsystem>/<name>}
   */
  public GenericRoller(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      GenericRollerConstants constants,
      String name) {
    this(
        motorFactory,
        subsystem,
        constants,
        name,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  /**
   * Builds the physics model for simulating a roller.
   *
   * @param constants the roller's constants
   * @return the physics model
   */
  public static MechanismSim simMechanism(GenericRollerConstants constants) {
    return new PlantMechanismSim(
        constants.rollerGearbox, constants.motor.gearRatio, constants.plant);
  }

  @Override
  public MechanismCore core() {
    return core;
  }

  @Override
  public OpenLoopControl<VoltageUnit> voltageControl() {
    return openLoop.voltage();
  }

  @Override
  public OpenLoopControl<CurrentUnit> currentControl() {
    return openLoop.current();
  }

  @Override
  public OpenLoopControl<DimensionlessUnit> dutyCycleControl() {
    return openLoop.dutyCycle();
  }

  @Override
  @Trace
  public void periodic() {
    core.periodic();
  }
}
