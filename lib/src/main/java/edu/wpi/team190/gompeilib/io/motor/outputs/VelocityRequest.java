package edu.wpi.team190.gompeilib.io.motor.outputs;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/** A closed-loop request toward a velocity, with or without a motion profile. */
public sealed interface VelocityRequest extends ClosedLoopRequest {
  /**
   * Returns the target velocity.
   *
   * @return the target velocity
   */
  AngularVelocity velocity();

  /**
   * Closes the loop on velocity, driving a duty-cycle output.
   *
   * @param velocity the target velocity
   * @param acceleration the acceleration feedforward target
   * @param feedforward the duty-cycle feedforward, in [-1, 1]
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record VelocityDutyCycle(
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      double feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options)
      implements VelocityRequest {
    public VelocityDutyCycle(AngularVelocity velocity) {
      this(
          velocity,
          RadiansPerSecondPerSecond.of(0.0),
          0.0,
          true,
          GainSlot.ZERO,
          RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.DUTY_CYCLE;
    }
  }

  /**
   * Closes the loop on velocity, driving a voltage output.
   *
   * @param velocity the target velocity
   * @param acceleration the acceleration feedforward target
   * @param feedforward the voltage feedforward
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record VelocityVoltage(
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Voltage feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options)
      implements VelocityRequest {
    public VelocityVoltage(AngularVelocity velocity) {
      this(
          velocity,
          RadiansPerSecondPerSecond.of(0.0),
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
   * Closes the loop on velocity, driving a torque-producing current output (always field-oriented).
   *
   * @param velocity the target velocity
   * @param acceleration the acceleration feedforward target
   * @param feedforward the torque-current feedforward
   * @param slot the gain slot
   * @param options the shared request options
   */
  record VelocityCurrent(
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Current feedforward,
      GainSlot slot,
      RequestOptions options)
      implements VelocityRequest {
    public VelocityCurrent(AngularVelocity velocity) {
      this(
          velocity,
          RadiansPerSecondPerSecond.of(0.0),
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
   * Moves to a velocity along an acceleration-limited motion profile, driving a duty-cycle output.
   *
   * @param velocity the target velocity
   * @param acceleration the profile's acceleration, or zero to use the configured value
   * @param feedforward the duty-cycle feedforward, in [-1, 1]
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record ProfiledVelocityDutyCycle(
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      double feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options)
      implements VelocityRequest {
    public ProfiledVelocityDutyCycle(AngularVelocity velocity) {
      this(
          velocity,
          RadiansPerSecondPerSecond.of(0.0),
          0.0,
          true,
          GainSlot.ZERO,
          RequestOptions.DEFAULT);
    }

    @Override
    public OutputType outputType() {
      return OutputType.DUTY_CYCLE;
    }
  }

  /**
   * Moves to a velocity along an acceleration-limited motion profile, driving a voltage output.
   *
   * @param velocity the target velocity
   * @param acceleration the profile's acceleration, or zero to use the configured value
   * @param feedforward the voltage feedforward
   * @param foc whether to use field-oriented control
   * @param slot the gain slot
   * @param options the shared request options
   */
  record ProfiledVelocityVoltage(
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Voltage feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options)
      implements VelocityRequest {
    public ProfiledVelocityVoltage(AngularVelocity velocity) {
      this(
          velocity,
          RadiansPerSecondPerSecond.of(0.0),
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
   * Moves to a velocity along an acceleration-limited motion profile, driving a torque-producing
   * current output (always field-oriented).
   *
   * @param velocity the target velocity
   * @param acceleration the profile's acceleration, or zero to use the configured value
   * @param feedforward the torque-current feedforward
   * @param slot the gain slot
   * @param options the shared request options
   */
  record ProfiledVelocityCurrent(
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Current feedforward,
      GainSlot slot,
      RequestOptions options)
      implements VelocityRequest {
    public ProfiledVelocityCurrent(AngularVelocity velocity) {
      this(
          velocity,
          RadiansPerSecondPerSecond.of(0.0),
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
   * Returns the velocity request for a profile mode and feedforward. The feedforward's kind picks
   * the output: voltage, torque current, or duty cycle.
   *
   * @param mode {@link ProfileMode#PROFILED} or {@link ProfileMode#UNPROFILED}
   * @param velocity the velocity, at the gearbox output
   * @param acceleration the acceleration, at the gearbox output
   * @param feedforward the feedforward, which picks the output type
   * @param foc whether voltage and duty-cycle requests use field-oriented control
   * @param slot the gain slot
   * @param options the request options
   * @return the request
   * @throws IllegalArgumentException if the mode is {@link ProfileMode#EXPONENTIAL}
   */
  static VelocityRequest of(
      ProfileMode mode,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      Feedforward feedforward,
      boolean foc,
      GainSlot slot,
      RequestOptions options) {
    if (mode == ProfileMode.EXPONENTIAL) {
      throw new IllegalArgumentException("Velocity control has no exponential profile");
    }
    boolean profiled = mode == ProfileMode.PROFILED;
    return switch (feedforward) {
      case Feedforward.Volts(Voltage ff) ->
          profiled
              ? new ProfiledVelocityVoltage(velocity, acceleration, ff, foc, slot, options)
              : new VelocityVoltage(velocity, acceleration, ff, foc, slot, options);
      case Feedforward.Amps(Current ff) ->
          profiled
              ? new ProfiledVelocityCurrent(velocity, acceleration, ff, slot, options)
              : new VelocityCurrent(velocity, acceleration, ff, slot, options);
      case Feedforward.DutyCycle(double ff) ->
          profiled
              ? new ProfiledVelocityDutyCycle(velocity, acceleration, ff, foc, slot, options)
              : new VelocityDutyCycle(velocity, acceleration, ff, foc, slot, options);
    };
  }
}
