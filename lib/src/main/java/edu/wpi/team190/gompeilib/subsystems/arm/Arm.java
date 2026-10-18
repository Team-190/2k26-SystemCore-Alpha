package edu.wpi.team190.gompeilib.subsystems.arm;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.physics.ArmMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import edu.wpi.team190.gompeilib.logging.Trace;
import java.util.Optional;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * A position-controlled arm: one leader motor, any number of followers, and an optional absolute
 * encoder on the arm's output that can seed the motors' position. An arm with {@link
 * ArmConstants#homing} can instead find its position by driving into a hard stop.
 */
public class Arm {
  private static final Voltage VOLTAGE_TOLERANCE = Millivolts.of(500);
  // Homing drives past the soft limits to reach the hard stop
  private static final RequestOptions HOMING_OPTIONS =
      new RequestOptions(
          false, false, false, false, true, false, RequestOptions.DEFAULT_UPDATE_FREQUENCY);

  private final ArmConstants constants;
  private final MotorControllerGroup motors;
  private final Optional<AbsoluteEncoderIO> encoder;
  private final AbsoluteEncoderTelemetryAutoLogged encoderTelemetry =
      new AbsoluteEncoderTelemetryAutoLogged();

  private final String aKitTopic;

  @Getter private ArmState currentState;

  @Getter private Setpoint<VoltageUnit> voltageGoal;
  @Getter private Setpoint<AngleUnit> positionGoal;
  @Getter private GainSlot gainSlot = GainSlot.ZERO;

  private final SysIdRoutine characterizationRoutine;

  /**
   * Creates an arm.
   *
   * @param motorFactory how to build the arm's motors
   * @param encoder the absolute encoder on the arm's output, or empty for none
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param index the arm's index, appended to the log prefix
   * @param constants the arm's constants
   * @param positionGoal the initial position goal
   * @param voltageGoal the initial voltage goal
   */
  public Arm(
      MotorIOFactory motorFactory,
      Optional<AbsoluteEncoderIO> encoder,
      Subsystem subsystem,
      int index,
      ArmConstants constants,
      Setpoint<AngleUnit> positionGoal,
      Setpoint<VoltageUnit> voltageGoal) {
    this.constants = constants;
    motors = new MotorControllerGroup(motorConfig(constants), motorFactory);
    this.encoder = encoder;

    aKitTopic = subsystem.getName() + "/Arms" + index;

    currentState = ArmState.IDLE;

    this.positionGoal = positionGoal;
    this.voltageGoal = voltageGoal;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(9),
                Seconds.of(12),
                (state) -> Logger.recordOutput(aKitTopic + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(this::setVoltage, null, subsystem));

    constants.motor.slots.forEach(
        (slot, slotConfig) -> slotConfig.gains().onChange(gains -> updateGains(gains, slot)));
    constants.constraints.onChange(this::updateConstraints);
  }

  /**
   * Creates an arm without an absolute encoder, with a position goal at zero limited to its travel
   * and a 0 V goal.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param index the arm's index, appended to the log prefix
   * @param constants the arm's constants
   */
  public Arm(MotorIOFactory motorFactory, Subsystem subsystem, int index, ArmConstants constants) {
    this(
        motorFactory,
        Optional.empty(),
        subsystem,
        index,
        constants,
        new Setpoint<>(
            Rotation2d.ZERO.getMeasure(),
            constants.positionOffsetStep.getMeasure(),
            constants.armParameters.minAngle().getMeasure(),
            constants.armParameters.maxAngle().getMeasure()),
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  /**
   * Creates an arm without an absolute encoder, with a 0 V goal.
   *
   * @param motorFactory how to build the arm's motors
   * @param subsystem the subsystem that owns the arm, for its log prefix
   * @param index the arm's index, appended to the log prefix
   * @param constants the arm's constants
   * @param positionGoal the initial position goal
   */
  public Arm(
      MotorIOFactory motorFactory,
      Subsystem subsystem,
      int index,
      ArmConstants constants,
      Setpoint<AngleUnit> positionGoal) {
    this(
        motorFactory,
        Optional.empty(),
        subsystem,
        index,
        constants,
        positionGoal,
        new Setpoint<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12)));
  }

  /**
   * Builds the physics model for simulating an arm. It starts at its lowest angle.
   *
   * @param constants the arm's constants
   * @return the physics model
   */
  public static MechanismSim simMechanism(ArmConstants constants) {
    ArmConstants.ArmParameters parameters = constants.armParameters;
    return new ArmMechanismSim(
        parameters.armGearbox(),
        constants.motor.gearRatio,
        parameters.momentOfInertia(),
        parameters.length(),
        parameters.minAngle().getMeasure(),
        parameters.maxAngle().getMeasure(),
        parameters.simulateGravity(),
        parameters.minAngle().getMeasure());
  }

  private static MotorConfig motorConfig(ArmConstants constants) {
    return constants.motor.toBuilder()
        .withMotionProfile(motionProfile(constants.constraints))
        .build();
  }

  private static MotionProfileConfig motionProfile(AngularPositionConstraints constraints) {
    return MotionProfileConfig.trapezoidal(
        constraints.maxVelocity().get(), constraints.maxAcceleration().get());
  }

  private void setVoltage(Voltage voltage) {
    setVoltage(voltage, RequestOptions.DEFAULT);
  }

  private void setVoltage(Voltage voltage, RequestOptions options) {
    motors.setControl(
        new OpenLoopRequest.VoltageOutput(voltage, constants.motor.enableFOC, options));
  }

  @Trace
  public void periodic() {
    motors.periodic(aKitTopic);
    encoder.ifPresent(
        io -> {
          io.updateInputs(encoderTelemetry);
          Logger.processInputs(aKitTopic + "/Encoder", encoderTelemetry);
        });

    Logger.recordOutput(aKitTopic + "/State", currentState.name());
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Goal", voltageGoal.getSetpoint());
    Logger.recordOutput(
        aKitTopic + "/Position Goal", new Rotation2d((Angle) positionGoal.getSetpoint()));
    Logger.recordOutputMeasure(aKitTopic + "/Voltage Offset", voltageGoal.getOffset());
    Logger.recordOutputMeasure(aKitTopic + "/Position Offset", positionGoal.getOffset());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/At Position Goal", atPositionGoal());

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> setVoltage((Voltage) voltageGoal.getNewSetpoint());
      case CLOSED_LOOP_POSITION_CONTROL ->
          motors.setControl(
              new PositionRequest.ProfiledPositionVoltage(
                  (Angle) positionGoal.getNewSetpoint(),
                  Volts.of(0.0),
                  constants.motor.enableFOC,
                  gainSlot,
                  RequestOptions.DEFAULT));
      case IDLE -> {}
    }
  }

  public Rotation2d getArmPosition() {
    return new Rotation2d(motors.leader().position);
  }

  public AngularVelocity getVelocity() {
    return motors.leader().velocity;
  }

  public Current getTorqueCurrent() {
    return motors.leader().torqueCurrent;
  }

  public void setVoltageGoal(Voltage voltageGoal) {
    currentState = ArmState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal.setSetpoint(voltageGoal);
  }

  public void setVoltageGoal(Setpoint<VoltageUnit> voltageGoal) {
    currentState = ArmState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setPositionGoal(Rotation2d positionGoal) {
    currentState = ArmState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal.setSetpoint(positionGoal.getMeasure());
  }

  public void setPositionGoal(Setpoint<AngleUnit> positionGoal) {
    currentState = ArmState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal = positionGoal;
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return motors.leader().appliedVoltage.isNear(voltageReference, VOLTAGE_TOLERANCE);
  }

  public boolean atPositionGoal(Rotation2d positionReference) {
    return Math.abs(motors.leader().position.in(Rotations) - positionReference.getRotations())
        < constants.constraints.goalTolerance().get().in(Rotations);
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal((Voltage) voltageGoal.getNewSetpoint());
  }

  public boolean atPositionGoal() {
    return atPositionGoal(new Rotation2d((Angle) positionGoal.getNewSetpoint()));
  }

  public void setPosition(Rotation2d position) {
    motors.setPosition(position.getMeasure());
  }

  /**
   * Resets the motors' position to the absolute encoder's latest reading. Does nothing without an
   * encoder.
   */
  public void resetPositionFromEncoder() {
    encoder.ifPresent(io -> motors.setPosition(encoderTelemetry.absolutePosition));
  }

  /**
   * Sets the gain slot position control uses from the next cycle on.
   *
   * @param gainSlot the slot
   */
  public void setGainSlot(GainSlot gainSlot) {
    this.gainSlot = gainSlot;
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atPositionGoal);
  }

  public void updateGains(Gains gains, GainSlot slot) {
    motors.updateGains(gains, slot);
  }

  public void updateConstraints(AngularPositionConstraints constraints) {
    motors.updateMotionProfile(motionProfile(constraints));
  }

  /**
   * Homes the arm: drives it into its hard stop, ignoring soft limits, until the leader's torque
   * current reaches the homing threshold, then stops it and sets its position to the stop's angle.
   *
   * @return the homing command
   * @throws IllegalStateException if the arm has no {@link ArmConstants#homing} config
   */
  public Command home() {
    ArmConstants.Homing homing =
        constants.homing.orElseThrow(
            () -> new IllegalStateException(aKitTopic + " has no homing config"));
    return Commands.sequence(
        Commands.runOnce(() -> currentState = ArmState.IDLE),
        Commands.run(() -> setVoltage(homing.voltage(), HOMING_OPTIONS))
            .until(
                () ->
                    getTorqueCurrent().isNear(homing.currentThreshold(), homing.currentEpsilon())),
        Commands.runOnce(
            () -> {
              setVoltage(Volts.zero());
              setPosition(homing.position());
            }));
  }

  public Command sysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = ArmState.IDLE),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.FORWARD),
        Commands.waitSeconds(1.0),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.REVERSE),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.FORWARD),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.REVERSE));
  }
}
