package edu.wpi.team190.gompeilib.core.io.motor.request;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
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
}
