package edu.wpi.team190.gompeilib.subsystems.roller;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Millivolts;
import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.physics.DCMotorMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Voltage;

/** A voltage-driven roller: one leader motor and any number of followers. */
public class GenericRoller {
  private static final Voltage VOLTAGE_TOLERANCE = Millivolts.of(500);

  private final GenericRollerConstants constants;
  private final MotorControllerGroup motors;

  private final String aKitTopic;

  @Getter private Setpoint<VoltageUnit> voltageGoal;

  /**
   * Creates a roller.
   *
   * @param motorFactory how to build the roller's motors
   * @param subsystem the subsystem that owns the roller, for its log prefix
   * @param constants the roller's constants
   * @param name the roller's name, appended to the log prefix
   * @param voltageGoal the initial voltage goal
   */
  public GenericRoller(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      GenericRollerConstants constants,
      String name,
      Setpoint<VoltageUnit> voltageGoal) {
    this.constants = constants;
    motors = new MotorControllerGroup(constants.motor, motorFactory);
    aKitTopic = subsystem.getName() + "/Roller" + name;
    this.voltageGoal = voltageGoal;
  }

  /**
   * Creates a roller with a 0 V goal, limited to ±12 V.
   *
   * @param motorFactory how to build the roller's motors
   * @param subsystem the subsystem that owns the roller, for its log prefix
   * @param constants the roller's constants
   * @param name the roller's name, appended to the log prefix
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
    return new DCMotorMechanismSim(
        constants.rollerGearbox, constants.momentOfInertia, constants.motor.gearRatio);
  }

  @Trace
  public void periodic() {
    motors.periodic(aKitTopic);

    Logger.recordOutputMeasure(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());

    motors.setControl(
        new OpenLoopRequest.VoltageOutput(
            (Voltage) voltageGoal.getNewSetpoint(),
            constants.motor.enableFOC,
            RequestOptions.DEFAULT));
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return motors.leader().appliedVoltage.isNear(voltageReference, VOLTAGE_TOLERANCE);
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public void setVoltageGoal(Voltage voltage) {
    voltageGoal.setSetpoint(voltage);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltage) {
    voltageGoal = voltage;
  }

  /**
   * Returns each motor's torque current in amps, leader first.
   *
   * @return the torque currents
   */
  public double[] getTorqueCurrent() {
    List<MotorTelemetry> telemetry = motors.telemetry();
    double[] currents = new double[telemetry.size()];
    for (int i = 0; i < currents.length; i++) {
      currents[i] = telemetry.get(i).torqueCurrent.in(Amps);
    }
    return currents;
  }
}
