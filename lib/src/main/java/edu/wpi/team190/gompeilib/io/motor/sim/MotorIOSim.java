package edu.wpi.team190.gompeilib.io.motor.sim;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorControlMode;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.inputs.RotorPolarity;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.Differential;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import java.util.EnumMap;
import java.util.Map;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.ExponentialProfile;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.system.RobotController;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.MultUnit;
import org.wpilib.units.PerUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Per;

/**
 * A vendor-neutral simulated motor that runs requests in software against a {@link MechanismSim}.
 *
 * <p>Closed loops follow the same conventions as the hardware: gains are in output units per
 * rotation (or rotation per second) at the gearbox output, and {@code kS} takes the sign of the
 * velocity target. Like real hardware, the motor outputs nothing while the robot is disabled.
 *
 * <p>Soft limits, continuous wrap, and the torque-current peaks are applied; current limits are
 * not. Each cycle reports one odometry sample.
 *
 * <p>Simplifications: profile jerk and the exponential profile's velocity limit are ignored; coast
 * and brake both apply zero volts; differential requests are unsupported because a single {@link
 * MechanismSim} has one axis.
 */
public class MotorIOSim implements MotorIO {
  private static final double EPSILON = 1e-9;

  private final MotorConfig config;
  private final MechanismSim mechanism;
  private final boolean driver;
  private final Map<GainSlot, SlotGains> slots = new EnumMap<>(GainSlot.class);
  private MotionProfileConfig motionProfile;

  private Command command = new Command.Zero();
  private MotorControlMode mode = MotorControlMode.NEUTRAL;
  private boolean foc = false;
  private boolean resetLoop = true;

  private double positionOffsetRotations = 0.0;
  private double integral = 0.0;
  private double previousError = 0.0;
  private double setpointPosition = 0.0;
  private double setpointVelocity = 0.0;

  private double appliedVolts = 0.0;
  private double reference = 0.0;
  private double referenceSlope = 0.0;
  private double error = 0.0;
  private double proportional = 0.0;
  private double integrated = 0.0;
  private double derivative = 0.0;
  private double feedforward = 0.0;
  private double output = 0.0;
  private boolean profileRunning = false;
  private boolean profileAtTarget = false;

  /**
   * Creates a simulated motor.
   *
   * @param config the motor's config
   * @param mechanism the physics model it drives or reads
   */
  public MotorIOSim(MotorConfig config, MechanismSim mechanism) {
    this.config = config;
    this.mechanism = mechanism;
    this.driver = mechanism.claimDriver(this);
    this.motionProfile = config.motionProfile;
    for (Map.Entry<GainSlot, SlotConfig> slot : config.slots.entrySet()) {
      slots.put(slot.getKey(), SlotGains.of(slot.getValue()));
    }
  }

  /**
   * Returns a factory for simulated motors sharing one mechanism. Motors from the same factory on
   * the same bus hardware-follow each other.
   *
   * @param mechanism the physics model
   * @return the factory
   */
  public static MotorIOFactory factory(MechanismSim mechanism) {
    return new MotorIOFactory() {
      @Override
      public MotorIO create(MotorConfig config) {
        return new MotorIOSim(config, mechanism);
      }

      @Override
      public String family() {
        return "Sim@" + System.identityHashCode(mechanism);
      }

      @Override
      public boolean supportsHardwareFollowing() {
        return true;
      }
    };
  }

  /**
   * Returns whether this motor drives its mechanism, rather than reading it.
   *
   * @return true if this motor drives the mechanism
   */
  public boolean isDriver() {
    return driver;
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public void setControl(MotorRequest request) {
    Command next = translate(request);
    boolean sameLoop = next.sameLoopAs(command) && next.slot() == command.slot();
    resetLoop |= !sameLoop;
    command = next;
  }

  @Override
  public void setPosition(Angle position) {
    positionOffsetRotations = position.in(Rotations) - mechanism.position().in(Rotations);
  }

  @Override
  public void updateGains(SlotConfig gains, GainSlot gainSlot) {
    slots.put(gainSlot, SlotGains.of(gains));
  }

  @Override
  public void updateMotionProfile(MotionProfileConfig motionProfile) {
    this.motionProfile = motionProfile;
  }

  @Override
  public void updateInputs(MotorTelemetry telemetry) {
    double dt = GompeiLib.getLoopPeriod();
    double battery = RobotController.getBatteryVoltage();
    boolean enabled = RobotState.isEnabled();

    run(dt, battery);
    if (!enabled) {
      appliedVolts = 0.0;
    }
    if (driver) {
      mechanism.setInputVoltage(Volts.of(appliedVolts));
      mechanism.update(Seconds.of(dt));
    }
    fill(telemetry, battery, enabled);
  }

  private double positionRotations() {
    return mechanism.position().in(Rotations) + positionOffsetRotations;
  }

  private double velocityRps() {
    return mechanism.velocity().in(RotationsPerSecond);
  }

  private void run(double dt, double battery) {
    clearLoopTerms();
    switch (command) {
      case Command.Open open -> appliedVolts = toVolts(open.output(), open.value(), battery);
      case Command.Position position -> runPosition(position, dt, battery);
      case Command.Velocity velocity -> runVelocity(velocity, dt, battery);
      case Command.Follow follow ->
          appliedVolts = driver ? 0.0 : follow.sign() * mechanism.inputVoltage().in(Volts);
      case Command.Zero _ -> appliedVolts = 0.0;
    }
    applySoftLimits();
    resetLoop = false;
  }

  private void applySoftLimits() {
    double position = positionRotations();
    boolean pastForward =
        config.softLimits.forward().map(limit -> position >= limit.in(Rotations)).orElse(false);
    boolean pastReverse =
        config.softLimits.reverse().map(limit -> position <= limit.in(Rotations)).orElse(false);
    if ((pastForward && appliedVolts > 0) || (pastReverse && appliedVolts < 0)) {
      appliedVolts = 0.0;
    }
  }

  private void clearLoopTerms() {
    reference = 0.0;
    referenceSlope = 0.0;
    error = 0.0;
    proportional = 0.0;
    integrated = 0.0;
    derivative = 0.0;
    feedforward = 0.0;
    output = 0.0;
    profileRunning = false;
    profileAtTarget = false;
  }

  private void runPosition(Command.Position command, double dt, double battery) {
    double position = positionRotations();
    if (resetLoop) {
      setpointPosition = position;
      setpointVelocity = velocityRps();
    }
    // With continuous wrap, aim for the copy of the target nearest the current position.
    double target =
        config.continuousWrap
            ? position + Math.IEEEremainder(command.target() - position, 1.0)
            : command.target();
    double velocityTarget = command.velocity();
    switch (command.profile()) {
      case Profile.None _ -> {
        setpointPosition = target;
        setpointVelocity = velocityTarget;
      }
      case Profile.Trapezoid trapezoid -> {
        profileRunning = true;
        if (trapezoid.cruiseVelocity() <= 0 || trapezoid.acceleration() <= 0) {
          setpointPosition = target;
          setpointVelocity = 0.0;
        } else {
          TrapezoidProfile.State next =
              new TrapezoidProfile(
                      new TrapezoidProfile.Constraints(
                          trapezoid.cruiseVelocity(), trapezoid.acceleration()))
                  .calculate(
                      dt,
                      new TrapezoidProfile.State(setpointPosition, setpointVelocity),
                      new TrapezoidProfile.State(target, 0.0));
          setpointPosition = next.position;
          setpointVelocity = next.velocity;
        }
        velocityTarget = setpointVelocity;
      }
      case Profile.Exponential exponential -> {
        profileRunning = true;
        if (exponential.kV() <= 0 || exponential.kA() <= 0) {
          setpointPosition = target;
          setpointVelocity = 0.0;
        } else {
          ExponentialProfile.State next =
              new ExponentialProfile(
                      ExponentialProfile.Constraints.fromCharacteristics(
                          battery, exponential.kV(), exponential.kA()))
                  .calculate(
                      dt,
                      new ExponentialProfile.State(setpointPosition, setpointVelocity),
                      new ExponentialProfile.State(target, 0.0));
          setpointPosition = next.position;
          setpointVelocity = next.velocity;
        }
        velocityTarget = setpointVelocity;
      }
    }
    // Both profiles come to rest exactly at the goal, so reaching it means the slope is zero.
    profileAtTarget = profileRunning && Math.abs(setpointPosition - target) < EPSILON;

    reference = setpointPosition;
    referenceSlope = velocityTarget;
    SlotGains gains = gains(command.slot());
    closeLoop(
        gains,
        reference - position,
        dt,
        gains.kS() * Math.signum(velocityTarget)
            + gains.kV() * velocityTarget
            + gains.kA() * command.acceleration()
            + gains.gravity(position)
            + command.feedforward());
    appliedVolts = toVolts(command.output(), output, battery);
  }

  private void runVelocity(Command.Velocity command, double dt, double battery) {
    if (resetLoop) {
      setpointVelocity = velocityRps();
    }
    double acceleration = command.acceleration();
    if (Double.isNaN(command.profileAcceleration())) {
      setpointVelocity = command.target();
    } else {
      profileRunning = true;
      double limit = command.profileAcceleration();
      double step = limit <= 0 ? Double.POSITIVE_INFINITY : limit * dt;
      double remaining = command.target() - setpointVelocity;
      setpointVelocity += Math.copySign(Math.min(Math.abs(remaining), step), remaining);
      acceleration =
          Math.abs(remaining) > EPSILON && limit > 0 ? Math.copySign(limit, remaining) : 0;
      profileAtTarget = Math.abs(setpointVelocity - command.target()) < EPSILON;
    }

    reference = setpointVelocity;
    referenceSlope = acceleration;
    SlotGains gains = gains(command.slot());
    closeLoop(
        gains,
        reference - velocityRps(),
        dt,
        gains.kS() * Math.signum(setpointVelocity)
            + gains.kV() * setpointVelocity
            + gains.kA() * acceleration
            + gains.gravity(positionRotations())
            + command.feedforward());
    appliedVolts = toVolts(command.output(), output, battery);
  }

  private void closeLoop(SlotGains gains, double loopError, double dt, double feedforwardTerm) {
    if (resetLoop) {
      integral = 0.0;
      previousError = loopError;
    }
    integral += loopError * dt;
    error = loopError;
    proportional = gains.kP() * loopError;
    integrated = gains.kI() * integral;
    derivative = gains.kD() * (loopError - previousError) / dt;
    previousError = loopError;
    feedforward = feedforwardTerm;
    output = proportional + integrated + derivative + feedforward;
  }

  private SlotGains gains(GainSlot slot) {
    return slots.getOrDefault(slot, SlotGains.ZERO);
  }

  private double toVolts(OutputType type, double value, double battery) {
    double volts =
        switch (type) {
          case DUTY_CYCLE -> value * battery;
          case VOLTAGE -> value;
          case CURRENT -> {
            double amps =
                Math.clamp(
                    value,
                    config.torqueCurrentLimits.peakReverse().in(Amps),
                    config.torqueCurrentLimits.peakForward().in(Amps));
            DCMotor gearbox = mechanism.gearbox();
            double rotorRadPerSec = mechanism.velocity().in(RadiansPerSecond) * config.gearRatio;
            yield gearbox.getVoltage(
                gearbox.getTorque(amps * mechanism.motorCount()), rotorRadPerSec);
          }
        };
    return Math.clamp(volts, -battery, battery);
  }

  private void fill(MotorTelemetry telemetry, double battery, boolean enabled) {
    double gearRatio = config.gearRatio;
    Angle position = Rotations.of(positionRotations());
    DCMotor gearbox = mechanism.gearbox();
    double motorCurrent = mechanism.currentDraw().in(Amps) / mechanism.motorCount();
    double dutyCycle = battery == 0 ? 0.0 : appliedVolts / battery;

    telemetry.connected = true;
    telemetry.position = position;
    telemetry.velocity = mechanism.velocity();
    telemetry.acceleration = mechanism.acceleration();
    telemetry.rotorPosition = position.times(gearRatio);
    telemetry.rotorVelocity = mechanism.velocity().times(gearRatio);
    telemetry.rotorAcceleration = mechanism.acceleration().times(gearRatio);
    telemetry.appliedVoltage = Volts.of(appliedVolts);
    telemetry.supplyVoltage = Volts.of(battery);
    telemetry.dutyCycle = dutyCycle;
    telemetry.outputCurrent = Amps.of(motorCurrent);
    telemetry.torqueCurrent = Amps.of(motorCurrent);
    telemetry.supplyCurrent = Amps.of(Math.abs(motorCurrent * dutyCycle));
    telemetry.rotorPolarity =
        config.direction == MotorDirection.CLOCKWISE_POSITIVE
            ? RotorPolarity.CLOCKWISE_POSITIVE
            : RotorPolarity.COUNTER_CLOCKWISE_POSITIVE;
    telemetry.motorKT = NewtonMeters.per(Amp).ofNative(gearbox.Kt);
    telemetry.motorKV = RadiansPerSecond.per(Volt).ofNative(gearbox.Kv);
    telemetry.motorStallCurrent = Amps.of(gearbox.stallCurrent / mechanism.motorCount());
    telemetry.robotEnabled = enabled;
    telemetry.deviceEnabled = enabled;
    telemetry.controlMode = enabled ? mode : MotorControlMode.DISABLED;
    telemetry.focActive = enabled && foc;
    telemetry.gainSlot = command.slot();

    boolean positionLoop = command instanceof Command.Position;
    boolean velocityLoop = command instanceof Command.Velocity;
    telemetry.closedLoopPositionReference = Rotations.of(positionLoop ? reference : 0.0);
    telemetry.closedLoopPositionReferenceSlope =
        RotationsPerSecond.of(positionLoop ? referenceSlope : 0.0);
    telemetry.closedLoopPositionError = Rotations.of(positionLoop ? error : 0.0);
    telemetry.closedLoopVelocityReference = RotationsPerSecond.of(velocityLoop ? reference : 0.0);
    telemetry.closedLoopVelocityReferenceSlope =
        RotationsPerSecondPerSecond.of(velocityLoop ? referenceSlope : 0.0);
    telemetry.closedLoopVelocityError = RotationsPerSecond.of(velocityLoop ? error : 0.0);
    telemetry.closedLoopProportionalOutput = proportional;
    telemetry.closedLoopIntegratedOutput = integrated;
    telemetry.closedLoopDerivativeOutput = derivative;
    telemetry.closedLoopFeedforward = feedforward;
    telemetry.closedLoopOutput = output;
    telemetry.profileRunning = profileRunning;
    telemetry.profileAtTarget = profileAtTarget;
    telemetry.odometryTimestamps = new double[] {RobotController.getTime() / 1e6};
    telemetry.odometryPositions = new Angle[] {position};
  }

  private Command translate(MotorRequest request) {
    double cruise = motionProfile.cruiseVelocity().in(RotationsPerSecond);
    double accel = motionProfile.acceleration().in(RotationsPerSecondPerSecond);
    double expoKV = motionProfile.exponentialKV().in(Volts.per(RotationsPerSecond));
    double expoKA = motionProfile.exponentialKA().in(Volts.per(RotationsPerSecondPerSecond));
    Profile trapezoid = new Profile.Trapezoid(cruise, accel);
    Profile exponential = new Profile.Exponential(expoKV, expoKA);

    return switch (request) {
      case OpenLoopRequest.DutyCycleOutput r -> {
        setMode(MotorControlMode.DUTY_CYCLE_OUTPUT, r.foc());
        yield new Command.Open(OutputType.DUTY_CYCLE, r.output());
      }
      case OpenLoopRequest.VoltageOutput r -> {
        setMode(MotorControlMode.VOLTAGE_OUTPUT, r.foc());
        yield new Command.Open(OutputType.VOLTAGE, r.output().in(Volts));
      }
      case OpenLoopRequest.CurrentOutput r -> {
        setMode(MotorControlMode.CURRENT_OUTPUT, true);
        yield new Command.Open(OutputType.CURRENT, r.output().in(Amps));
      }
      case PositionRequest.PositionDutyCycle r -> {
        setMode(MotorControlMode.POSITION_DUTY_CYCLE, r.foc());
        yield position(r, r.velocity().in(RotationsPerSecond), r.feedforward(), new Profile.None());
      }
      case PositionRequest.PositionVoltage r -> {
        setMode(MotorControlMode.POSITION_VOLTAGE, r.foc());
        yield position(
            r, r.velocity().in(RotationsPerSecond), r.feedforward().in(Volts), new Profile.None());
      }
      case PositionRequest.PositionCurrent r -> {
        setMode(MotorControlMode.POSITION_CURRENT, true);
        yield position(
            r, r.velocity().in(RotationsPerSecond), r.feedforward().in(Amps), new Profile.None());
      }
      case PositionRequest.ProfiledPositionDutyCycle r -> {
        setMode(MotorControlMode.PROFILED_POSITION_DUTY_CYCLE, r.foc());
        yield position(r, 0.0, r.feedforward(), trapezoid);
      }
      case PositionRequest.ProfiledPositionVoltage r -> {
        setMode(MotorControlMode.PROFILED_POSITION_VOLTAGE, r.foc());
        yield position(r, 0.0, r.feedforward().in(Volts), trapezoid);
      }
      case PositionRequest.ProfiledPositionCurrent r -> {
        setMode(MotorControlMode.PROFILED_POSITION_CURRENT, true);
        yield position(r, 0.0, r.feedforward().in(Amps), trapezoid);
      }
      case PositionRequest.ExponentialProfiledPositionDutyCycle r -> {
        setMode(MotorControlMode.EXPONENTIAL_PROFILED_POSITION_DUTY_CYCLE, r.foc());
        yield position(r, 0.0, r.feedforward(), exponential);
      }
      case PositionRequest.ExponentialProfiledPositionVoltage r -> {
        setMode(MotorControlMode.EXPONENTIAL_PROFILED_POSITION_VOLTAGE, r.foc());
        yield position(r, 0.0, r.feedforward().in(Volts), exponential);
      }
      case PositionRequest.ExponentialProfiledPositionCurrent r -> {
        setMode(MotorControlMode.EXPONENTIAL_PROFILED_POSITION_CURRENT, true);
        yield position(r, 0.0, r.feedforward().in(Amps), exponential);
      }
      case PositionRequest.DynamicProfiledPositionDutyCycle r -> {
        setMode(MotorControlMode.PROFILED_POSITION_DUTY_CYCLE, r.foc());
        yield position(r, 0.0, r.feedforward(), trapezoid(r.velocity(), r.acceleration()));
      }
      case PositionRequest.DynamicProfiledPositionVoltage r -> {
        setMode(MotorControlMode.PROFILED_POSITION_VOLTAGE, r.foc());
        yield position(
            r, 0.0, r.feedforward().in(Volts), trapezoid(r.velocity(), r.acceleration()));
      }
      case PositionRequest.DynamicProfiledPositionCurrent r -> {
        setMode(MotorControlMode.PROFILED_POSITION_CURRENT, true);
        yield position(r, 0.0, r.feedforward().in(Amps), trapezoid(r.velocity(), r.acceleration()));
      }
      case PositionRequest.DynamicExponentialProfiledPositionDutyCycle r -> {
        setMode(MotorControlMode.EXPONENTIAL_PROFILED_POSITION_DUTY_CYCLE, r.foc());
        yield position(r, 0.0, r.feedforward(), exponential(r.kV(), r.kA()));
      }
      case PositionRequest.DynamicExponentialProfiledPositionVoltage r -> {
        setMode(MotorControlMode.EXPONENTIAL_PROFILED_POSITION_VOLTAGE, r.foc());
        yield position(r, 0.0, r.feedforward().in(Volts), exponential(r.kV(), r.kA()));
      }
      case PositionRequest.DynamicExponentialProfiledPositionCurrent r -> {
        setMode(MotorControlMode.EXPONENTIAL_PROFILED_POSITION_CURRENT, true);
        yield position(r, 0.0, r.feedforward().in(Amps), exponential(r.kV(), r.kA()));
      }
      case VelocityRequest.VelocityDutyCycle r -> {
        setMode(MotorControlMode.VELOCITY_DUTY_CYCLE, r.foc());
        yield velocity(r, r.acceleration(), r.feedforward(), Double.NaN);
      }
      case VelocityRequest.VelocityVoltage r -> {
        setMode(MotorControlMode.VELOCITY_VOLTAGE, r.foc());
        yield velocity(r, r.acceleration(), r.feedforward().in(Volts), Double.NaN);
      }
      case VelocityRequest.VelocityCurrent r -> {
        setMode(MotorControlMode.VELOCITY_CURRENT, true);
        yield velocity(r, r.acceleration(), r.feedforward().in(Amps), Double.NaN);
      }
      case VelocityRequest.ProfiledVelocityDutyCycle r -> {
        setMode(MotorControlMode.PROFILED_VELOCITY_DUTY_CYCLE, r.foc());
        yield velocity(r, null, r.feedforward(), profileAcceleration(r.acceleration(), accel));
      }
      case VelocityRequest.ProfiledVelocityVoltage r -> {
        setMode(MotorControlMode.PROFILED_VELOCITY_VOLTAGE, r.foc());
        yield velocity(
            r, null, r.feedforward().in(Volts), profileAcceleration(r.acceleration(), accel));
      }
      case VelocityRequest.ProfiledVelocityCurrent r -> {
        setMode(MotorControlMode.PROFILED_VELOCITY_CURRENT, true);
        yield velocity(
            r, null, r.feedforward().in(Amps), profileAcceleration(r.acceleration(), accel));
      }
      case FollowerRequest.Follow r -> {
        setMode(MotorControlMode.FOLLOWER, false);
        yield new Command.Follow(r.alignment() == MotorAlignment.ALIGNED ? 1.0 : -1.0);
      }
      case FollowerRequest.StrictFollow _ -> {
        setMode(MotorControlMode.FOLLOWER, false);
        yield new Command.Follow(1.0);
      }
      case FollowerRequest.DifferentialFollow r -> {
        setMode(MotorControlMode.FOLLOWER, false);
        yield new Command.Follow(r.alignment() == MotorAlignment.ALIGNED ? 1.0 : -1.0);
      }
      case FollowerRequest.DifferentialStrictFollow _ -> {
        setMode(MotorControlMode.FOLLOWER, false);
        yield new Command.Follow(1.0);
      }
      case NeutralRequest.Neutral _ -> zero(MotorControlMode.NEUTRAL);
      case NeutralRequest.Coast _ -> zero(MotorControlMode.COAST);
      case NeutralRequest.StaticBrake _ -> zero(MotorControlMode.STATIC_BRAKE);
      case NeutralRequest.Tone _ -> zero(MotorControlMode.TONE);
      case Differential r -> translate(r.average());
    };
  }

  private void setMode(MotorControlMode mode, boolean foc) {
    this.mode = mode;
    this.foc = foc;
  }

  private Command zero(MotorControlMode mode) {
    setMode(mode, false);
    return new Command.Zero();
  }

  private static Command.Position position(
      PositionRequest request, double velocity, double feedforward, Profile profile) {
    return new Command.Position(
        request.outputType(),
        request.position().in(Rotations),
        velocity,
        0.0,
        feedforward,
        request.slot(),
        profile);
  }

  private static Command.Velocity velocity(
      VelocityRequest request,
      AngularAcceleration acceleration,
      double feedforward,
      double profileAcceleration) {
    return new Command.Velocity(
        request.outputType(),
        request.velocity().in(RotationsPerSecond),
        acceleration == null ? 0.0 : acceleration.in(RotationsPerSecondPerSecond),
        feedforward,
        request.slot(),
        profileAcceleration);
  }

  private static double profileAcceleration(AngularAcceleration requested, double configured) {
    double value = requested.in(RotationsPerSecondPerSecond);
    return value > 0 ? value : configured;
  }

  private static Profile trapezoid(AngularVelocity velocity, AngularAcceleration acceleration) {
    return new Profile.Trapezoid(
        velocity.in(RotationsPerSecond), acceleration.in(RotationsPerSecondPerSecond));
  }

  private static Profile exponential(
      Per<VoltageUnit, AngularVelocityUnit> kV, Per<VoltageUnit, AngularAccelerationUnit> kA) {
    return new Profile.Exponential(
        kV.in(Volts.per(RotationsPerSecond)), kA.in(Volts.per(RotationsPerSecondPerSecond)));
  }

  /** A slot's gains as plain numbers. */
  private record SlotGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      GravityType gravityType) {
    static final SlotGains ZERO = new SlotGains(0, 0, 0, 0, 0, 0, 0, GravityType.CONSTANT);

    /** Reads a slot in the units the simulation runs in: per rotation, or per rotation/s. */
    static SlotGains of(SlotConfig slot) {
      return switch (slot) {
        case SlotConfig.Position position -> position(position.gains(), slot.gravityType());
        case SlotConfig.Velocity velocity -> velocity(velocity.gains(), slot.gravityType());
      };
    }

    private static <O extends Unit> SlotGains position(
        Gains<O, AngleUnit, AngularVelocityUnit> gains, GravityType gravityType) {
      O output = gains.baseOutputUnit();
      return new SlotGains(
          gains.getKP(PerUnit.combine(output, Rotations)),
          gains.getKI(PerUnit.combine(output, MultUnit.combine(Rotations, Seconds))),
          gains.getKD(PerUnit.combine(output, RotationsPerSecond)),
          gains.getKS(output),
          gains.getKV(PerUnit.combine(output, RotationsPerSecond)),
          gains.getKA(PerUnit.combine(output, RotationsPerSecondPerSecond)),
          gains.getKG(output),
          gravityType);
    }

    private static <O extends Unit> SlotGains velocity(
        Gains<O, AngularVelocityUnit, AngularVelocityUnit> gains, GravityType gravityType) {
      O output = gains.baseOutputUnit();
      return new SlotGains(
          gains.getKP(PerUnit.combine(output, RotationsPerSecond)),
          gains.getKI(PerUnit.combine(output, MultUnit.combine(RotationsPerSecond, Seconds))),
          gains.getKD(PerUnit.combine(output, RotationsPerSecondPerSecond)),
          gains.getKS(output),
          gains.getKV(PerUnit.combine(output, RotationsPerSecond)),
          gains.getKA(PerUnit.combine(output, RotationsPerSecondPerSecond)),
          gains.getKG(output),
          gravityType);
    }

    double gravity(double positionRotations) {
      return gravityType == GravityType.COSINE
          ? kG * Math.cos(positionRotations * 2.0 * Math.PI)
          : kG;
    }
  }

  /** How a position loop's setpoint moves toward its target. */
  private sealed interface Profile {
    record None() implements Profile {}

    record Trapezoid(double cruiseVelocity, double acceleration) implements Profile {}

    record Exponential(double kV, double kA) implements Profile {}
  }

  /** A request reduced to what the simulation needs. Units are rotations and output units. */
  private sealed interface Command {
    default GainSlot slot() {
      return GainSlot.ZERO;
    }

    /** Whether switching from {@code other} to this command continues the same closed loop. */
    default boolean sameLoopAs(Command other) {
      return false;
    }

    record Open(OutputType output, double value) implements Command {}

    record Position(
        OutputType output,
        double target,
        double velocity,
        double acceleration,
        double feedforward,
        GainSlot slot,
        Profile profile)
        implements Command {
      @Override
      public boolean sameLoopAs(Command other) {
        return other instanceof Position p && p.profile().getClass() == profile.getClass();
      }
    }

    record Velocity(
        OutputType output,
        double target,
        double acceleration,
        double feedforward,
        GainSlot slot,
        double profileAcceleration)
        implements Command {
      @Override
      public boolean sameLoopAs(Command other) {
        return other instanceof Velocity v
            && Double.isNaN(v.profileAcceleration()) == Double.isNaN(profileAcceleration);
      }
    }

    record Follow(double sign) implements Command {}

    record Zero() implements Command {}
  }
}
