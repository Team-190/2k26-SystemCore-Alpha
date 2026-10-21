package edu.wpi.team190.gompeilib.robot.mechanisms.roller;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Value;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.io.physics.PlantMechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.CurrentControllable;
import edu.wpi.team190.gompeilib.robot.mechanisms.DutyCycleControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.DutyCycleControllable;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControl;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControllable;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.VoltageUnit;

/**
 * A roller that can be driven in voltage, current, or duty cycle: one leader motor and any number
 * of followers. It starts driving its voltage goal, so it sends that goal every loop.
 */
public class GenericRoller
    implements VoltageControllable, CurrentControllable, DutyCycleControllable {
  private final MechanismCore core;
  private final VoltageControl voltageControl;
  private final CurrentControl currentControl;
  private final DutyCycleControl dutyCycleControl;

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
    boolean foc = constants.motor.enableFOC;
    voltageControl = new VoltageControl(core, voltageGoal, foc);
    currentControl =
        new CurrentControl(core, new Setpoint<>(Amps.of(0), constants.currentOffsetStep));
    dutyCycleControl =
        new DutyCycleControl(
            core,
            new Setpoint<>(Value.of(0), constants.dutyCycleOffsetStep, Value.of(-1), Value.of(1)),
            foc);
    core.activate(voltageControl);
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
  public VoltageControl voltageControl() {
    return voltageControl;
  }

  @Override
  public CurrentControl currentControl() {
    return currentControl;
  }

  @Override
  public DutyCycleControl dutyCycleControl() {
    return dutyCycleControl;
  }

  @Override
  @Trace
  public void periodic() {
    core.periodic();
  }
}
