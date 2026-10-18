package edu.wpi.team190.gompeilib.io.motor.outputs;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Per;
import org.wpilib.units.measure.Velocity;
import org.wpilib.units.measure.Voltage;

/** A closed-loop request toward a position, with or without a motion profile. */
public sealed interface PositionRequest extends ClosedLoopRequest {
  /**
   * Returns the target position.
   *
   * @return the target position
   */
  Angle position();

  /**
   * Closes the loop on position, driving a duty-cycle output.
   *
   * @param position the target position
   * @param velocity the velocity feedforward target
   * @param feedforward the duty-cycle feedforward, in [-1, 1]
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record PositionDutyCycle(
      Angle position,
      AngularVelocity velocity,
      double feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options)
      implements PositionRequest {
    public PositionDutyCycle(Angle position) {
      this(position, RadiansPerSecond.of(0.0), 0.0, true, GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.DUTY_CYCLE;
    }
  }

  /**
   * Closes the loop on position, driving a voltage output.
   *
   * @param position the target position
   * @param velocity the velocity feedforward target
   * @param feedforward the voltage feedforward
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record PositionVoltage(
      Angle position,
      AngularVelocity velocity,
      Voltage feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options)
      implements PositionRequest {
    public PositionVoltage(Angle position) {
      this(
          position,
          RadiansPerSecond.of(0.0),
          Volts.of(0.0),
          true,
          GainSlot.ZERO,
          RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.VOLTAGE;
    }
  }

  /**
   * Closes the loop on position, driving a torque-producing current output (always field-oriented).
   *
   * @param position the target position
   * @param velocity the velocity feedforward target
   * @param feedforward the torque-current feedforward
   * @param slot the gain slot
   * @param options the shared request options
   */
  record PositionCurrent(
      Angle position,
      AngularVelocity velocity,
      Current feedforward,
      GainSlot slot,
      RequestOptions options)
      implements PositionRequest {
    public PositionCurrent(Angle position) {
      this(position, RadiansPerSecond.of(0.0), Amps.of(0.0), GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.CURRENT;
    }
  }

  /**
   * Moves to a position along a trapezoidal motion profile, driving a duty-cycle output.
   *
   * @param position the target position
   * @param feedforward the duty-cycle feedforward, in [-1, 1]
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record ProfiledPositionDutyCycle(
      Angle position, double feedforward, boolean foc, GainSlot slot, RequestOptions options)
      implements PositionRequest {
    public ProfiledPositionDutyCycle(Angle position) {
      this(position, 0.0, true, GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.DUTY_CYCLE;
    }
  }

  /**
   * Moves to a position along a trapezoidal motion profile, driving a voltage output.
   *
   * @param position the target position
   * @param feedforward the voltage feedforward
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record ProfiledPositionVoltage(
      Angle position, Voltage feedforward, boolean foc, GainSlot slot, RequestOptions options)
      implements PositionRequest {
    public ProfiledPositionVoltage(Angle position) {
      this(position, Volts.of(0.0), true, GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.VOLTAGE;
    }
  }

  /**
   * Moves to a position along a trapezoidal motion profile, driving a torque-producing current
   * output (always field-oriented).
   *
   * @param position the target position
   * @param feedforward the torque-current feedforward
   * @param slot the gain slot
   * @param options the shared request options
   */
  record ProfiledPositionCurrent(
      Angle position, Current feedforward, GainSlot slot, RequestOptions options)
      implements PositionRequest {
    public ProfiledPositionCurrent(Angle position) {
      this(position, Amps.of(0.0), GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.CURRENT;
    }
  }

  /**
   * Moves to a position along an exponential motion profile, driving a duty-cycle output.
   *
   * @param position the target position
   * @param feedforward the duty-cycle feedforward, in [-1, 1]
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record ExponentialProfiledPositionDutyCycle(
      Angle position, double feedforward, boolean foc, GainSlot slot, RequestOptions options)
      implements PositionRequest {
    public ExponentialProfiledPositionDutyCycle(Angle position) {
      this(position, 0.0, true, GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.DUTY_CYCLE;
    }
  }

  /**
   * Moves to a position along an exponential motion profile, driving a voltage output.
   *
   * @param position the target position
   * @param feedforward the voltage feedforward
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record ExponentialProfiledPositionVoltage(
      Angle position, Voltage feedforward, boolean foc, GainSlot slot, RequestOptions options)
      implements PositionRequest {
    public ExponentialProfiledPositionVoltage(Angle position) {
      this(position, Volts.of(0.0), true, GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.VOLTAGE;
    }
  }

  /**
   * Moves to a position along an exponential motion profile, driving a torque-producing current
   * output (always field-oriented).
   *
   * @param position the target position
   * @param feedforward the torque-current feedforward
   * @param slot the gain slot
   * @param options the shared request options
   */
  record ExponentialProfiledPositionCurrent(
      Angle position, Current feedforward, GainSlot slot, RequestOptions options)
      implements PositionRequest {
    public ExponentialProfiledPositionCurrent(Angle position) {
      this(position, Amps.of(0.0), GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.CURRENT;
    }
  }

  /**
   * Moves to a position along a trapezoidal motion profile with per-request constraints, driving a
   * duty-cycle output.
   *
   * @param position the target position
   * @param velocity the profile's cruise velocity
   * @param acceleration the profile's acceleration
   * @param jerk the profile's jerk
   * @param feedforward the duty-cycle feedforward, in [-1, 1]
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record DynamicProfiledPositionDutyCycle(
      Angle position,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk,
      double feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options)
      implements PositionRequest {
    public DynamicProfiledPositionDutyCycle(
        Angle position,
        AngularVelocity velocity,
        AngularAcceleration acceleration,
        Velocity<AngularAccelerationUnit> jerk) {
      this(
          position, velocity, acceleration, jerk, 0.0, true, GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.DUTY_CYCLE;
    }
  }

  /**
   * Moves to a position along a trapezoidal motion profile with per-request constraints, driving a
   * voltage output.
   *
   * @param position the target position
   * @param velocity the profile's cruise velocity
   * @param acceleration the profile's acceleration
   * @param jerk the profile's jerk
   * @param feedforward the voltage feedforward
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record DynamicProfiledPositionVoltage(
      Angle position,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk,
      Voltage feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options)
      implements PositionRequest {
    public DynamicProfiledPositionVoltage(
        Angle position,
        AngularVelocity velocity,
        AngularAcceleration acceleration,
        Velocity<AngularAccelerationUnit> jerk) {
      this(
          position,
          velocity,
          acceleration,
          jerk,
          Volts.of(0.0),
          true,
          GainSlot.ZERO,
          RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.VOLTAGE;
    }
  }

  /**
   * Moves to a position along a trapezoidal motion profile with per-request constraints, driving a
   * torque-producing current output (always field-oriented).
   *
   * @param position the target position
   * @param velocity the profile's cruise velocity
   * @param acceleration the profile's acceleration
   * @param jerk the profile's jerk
   * @param feedforward the torque-current feedforward
   * @param slot the gain slot
   * @param options the shared request options
   */
  record DynamicProfiledPositionCurrent(
      Angle position,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> jerk,
      Current feedforward,
      GainSlot slot,
      RequestOptions options)
      implements PositionRequest {
    public DynamicProfiledPositionCurrent(
        Angle position,
        AngularVelocity velocity,
        AngularAcceleration acceleration,
        Velocity<AngularAccelerationUnit> jerk) {
      this(
          position,
          velocity,
          acceleration,
          jerk,
          Amps.of(0.0),
          GainSlot.ZERO,
          RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.CURRENT;
    }
  }

  /**
   * Moves to a position along an exponential motion profile with per-request constraints, driving a
   * duty-cycle output.
   *
   * @param position the target position
   * @param velocity the profile's cruise velocity, or zero for no limit
   * @param kV the profile's velocity gain
   * @param kA the profile's acceleration gain
   * @param feedforward the duty-cycle feedforward, in [-1, 1]
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record DynamicExponentialProfiledPositionDutyCycle(
      Angle position,
      AngularVelocity velocity,
      Per<VoltageUnit, AngularVelocityUnit> kV,
      Per<VoltageUnit, AngularAccelerationUnit> kA,
      double feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options)
      implements PositionRequest {
    public DynamicExponentialProfiledPositionDutyCycle(
        Angle position,
        AngularVelocity velocity,
        Per<VoltageUnit, AngularVelocityUnit> kV,
        Per<VoltageUnit, AngularAccelerationUnit> kA) {
      this(position, velocity, kV, kA, 0.0, true, GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.DUTY_CYCLE;
    }
  }

  /**
   * Moves to a position along an exponential motion profile with per-request constraints, driving a
   * voltage output.
   *
   * @param position the target position
   * @param velocity the profile's cruise velocity, or zero for no limit
   * @param kV the profile's velocity gain
   * @param kA the profile's acceleration gain
   * @param feedforward the voltage feedforward
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record DynamicExponentialProfiledPositionVoltage(
      Angle position,
      AngularVelocity velocity,
      Per<VoltageUnit, AngularVelocityUnit> kV,
      Per<VoltageUnit, AngularAccelerationUnit> kA,
      Voltage feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options)
      implements PositionRequest {
    public DynamicExponentialProfiledPositionVoltage(
        Angle position,
        AngularVelocity velocity,
        Per<VoltageUnit, AngularVelocityUnit> kV,
        Per<VoltageUnit, AngularAccelerationUnit> kA) {
      this(position, velocity, kV, kA, Volts.of(0.0), true, GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.VOLTAGE;
    }
  }

  /**
   * Moves to a position along an exponential motion profile with per-request constraints, driving a
   * torque-producing current output (always field-oriented).
   *
   * @param position the target position
   * @param velocity the profile's cruise velocity, or zero for no limit
   * @param kV the profile's velocity gain
   * @param kA the profile's acceleration gain
   * @param feedforward the torque-current feedforward
   * @param slot the gain slot
   * @param options the shared request options
   */
  record DynamicExponentialProfiledPositionCurrent(
      Angle position,
      AngularVelocity velocity,
      Per<VoltageUnit, AngularVelocityUnit> kV,
      Per<VoltageUnit, AngularAccelerationUnit> kA,
      Current feedforward,
      GainSlot slot,
      RequestOptions options)
      implements PositionRequest {
    public DynamicExponentialProfiledPositionCurrent(
        Angle position,
        AngularVelocity velocity,
        Per<VoltageUnit, AngularVelocityUnit> kV,
        Per<VoltageUnit, AngularAccelerationUnit> kA) {
      this(position, velocity, kV, kA, Amps.of(0.0), GainSlot.ZERO, RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.CURRENT;
    }
  }
}
