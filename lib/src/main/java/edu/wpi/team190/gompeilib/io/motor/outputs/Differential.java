package edu.wpi.team190.gompeilib.io.motor.outputs;

import java.util.Objects;
import org.wpilib.units.measure.Frequency;

/**
 * Drives a differential mechanism on two axes at once: the average of the two motors and the
 * difference between them. Each axis is an ordinary {@link AxisRequest}, so each one can be open or
 * closed loop independently.
 *
 * <p>The average may be any axis request except a dynamic profile. The difference must be open loop
 * or an unprofiled position or velocity loop. Both axes must use the same output type.
 *
 * @param average the request for the average axis
 * @param difference the request for the difference axis
 * @param updateFrequency how often the request is re-sent
 */
public record Differential(AxisRequest average, AxisRequest difference, Frequency updateFrequency)
    implements MotorRequest {

  public Differential {
    Objects.requireNonNull(average, "average");
    Objects.requireNonNull(difference, "difference");
    Objects.requireNonNull(updateFrequency, "updateFrequency");
    if (!isValidAverage(average)) {
      throw new IllegalArgumentException(
          average.getClass().getSimpleName() + " cannot drive the average axis");
    }
    if (!isValidDifference(difference)) {
      throw new IllegalArgumentException(
          difference.getClass().getSimpleName() + " cannot drive the difference axis");
    }
    if (average.outputType() != difference.outputType()) {
      throw new IllegalArgumentException(
          "average and difference must use the same output type, got "
              + average.outputType()
              + " and "
              + difference.outputType());
    }
  }

  public Differential(AxisRequest average, AxisRequest difference) {
    this(average, difference, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
  }

  /**
   * Checks whether a request can drive the average axis.
   *
   * @param request the request
   * @return true unless the request is a dynamic profile
   */
  public static boolean isValidAverage(AxisRequest request) {
    return !(request instanceof PositionRequest.DynamicProfiledPositionDutyCycle
        || request instanceof PositionRequest.DynamicProfiledPositionVoltage
        || request instanceof PositionRequest.DynamicProfiledPositionCurrent
        || request instanceof PositionRequest.DynamicExponentialProfiledPositionDutyCycle
        || request instanceof PositionRequest.DynamicExponentialProfiledPositionVoltage
        || request instanceof PositionRequest.DynamicExponentialProfiledPositionCurrent);
  }

  /**
   * Checks whether a request can drive the difference axis.
   *
   * @param request the request
   * @return true if the request is open loop or an unprofiled position or velocity loop
   */
  public static boolean isValidDifference(AxisRequest request) {
    return request instanceof OpenLoopRequest
        || request instanceof PositionRequest.PositionDutyCycle
        || request instanceof PositionRequest.PositionVoltage
        || request instanceof PositionRequest.PositionCurrent
        || request instanceof VelocityRequest.VelocityDutyCycle
        || request instanceof VelocityRequest.VelocityVoltage
        || request instanceof VelocityRequest.VelocityCurrent;
  }
}
