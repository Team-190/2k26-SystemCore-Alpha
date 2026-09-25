package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

public class Extension {
  public final ExtensionIO io;
  public final ExtensionIOInputsAutoLogged inputs;

  private final String aKitTopic;

  private ExtensionState currentState;

  private Setpoint<VoltageUnit> voltageGoal;
  private Setpoint<DistanceUnit> positionGoal;

  private final SysIdRoutine characterizationRoutine;

  public final ExtensionConstants constants;

  public Extension(
      ExtensionConstants constants,
      Subsystem subsystem,
      int index,
      ExtensionIO io,
      Setpoint<DistanceUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    this.io = io;
    this.inputs = new ExtensionIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/Extension" + index;

    currentState = ExtensionState.IDLE;

    this.positionGoal = positionGoal;
    this.voltageGoal = voltageGoal;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(3),
                Seconds.of(3),
                (state) -> Logger.recordOutput(aKitTopic + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltageGoal, null, subsystem));

    this.constants = constants;
  }

  public Extension(ExtensionConstants constants, Subsystem subsystem, int index, ExtensionIO io) {
    this(
        constants,
        subsystem,
        index,
        io,
        new Setpoint<>(
            Meters.of(0),
            constants.heightOffsetStep,
            constants.extensionParameters.MIN_LENGTH(),
            constants.extensionParameters.MAX_LENGTH()),
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  public Extension(
      ExtensionConstants constants,
      Subsystem subsystem,
      int index,
      ExtensionIO io,
      Setpoint<DistanceUnit> positionGoal) {
    this(
        constants,
        subsystem,
        index,
        io,
        positionGoal,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/State", currentState.name());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutputMeasure(aKitTopic + "/Position Goal", positionGoal.getSetpoint());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutputMeasure(aKitTopic + "/Position Offset", positionGoal.getOffset());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/At Position Goal", atPositionGoal());

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
      case CLOSED_LOOP_POSITION_CONTROL ->
          io.setPositionGoal((Distance) positionGoal.getNewSetpoint());
    }
  }

  public Distance getExtensionPosition() {
    return inputs.position;
  }

  public Current getTorqueCurrent() {
    return Amps.of(inputs.torqueCurrentAmps[0]);
  }

  public void setVoltageGoal(Voltage voltageGoal) {
    currentState = ExtensionState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal.setSetpoint(voltageGoal);
  }

  public void setPositionGoal(Distance positionGoal) {
    currentState = ExtensionState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal.setSetpoint(positionGoal);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltageGoal) {
    currentState = ExtensionState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setPositionGoal(Setpoint<DistanceUnit> positionGoal) {
    currentState = ExtensionState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal = positionGoal;
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return voltageGoal.getNewSetpoint().isNear(voltageReference, Millivolts.of(500));
  }

  public boolean atPositionGoal(Distance positionReference) {
    return positionGoal
        .getNewSetpoint()
        .isNear(positionReference, constants.constraints.goalTolerance().get());
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public boolean atPositionGoal() {
    return atPositionGoal((Distance) positionGoal.getNewSetpoint());
  }

  public void setPosition(Distance position) {
    io.setPosition(position);
  }

  public void setGainSlot(GainSlot gainSlot) {
    io.setGainSlot(gainSlot);
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atPositionGoal);
  }

  public void updateGains(Gains gains, GainSlot slot) {
    io.updateGains(gains, slot);
  }

  public void updateConstraints(LinearConstraints constraints) {
    io.updateConstraints(constraints);
  }

  public Command runSysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = ExtensionState.IDLE),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }
}
