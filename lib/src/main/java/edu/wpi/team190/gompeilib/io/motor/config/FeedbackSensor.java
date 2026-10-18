package edu.wpi.team190.gompeilib.io.motor.config;

import java.util.Objects;

/** Where a motor's closed loops read position and velocity from. */
public sealed interface FeedbackSensor {
  /** The motor's own rotor sensor, scaled by {@link MotorConfig#gearRatio}. */
  record Internal() implements FeedbackSensor {}

  /**
   * An absolute encoder on the gearbox output, on the motor's bus. {@link MotorConfig#gearRatio} is
   * then the rotor-to-encoder ratio.
   *
   * @param canId the encoder's CAN ID
   * @param mode how the encoder is used
   */
  record RemoteEncoder(int canId, RemoteSensorMode mode) implements FeedbackSensor {
    public RemoteEncoder {
      Objects.requireNonNull(mode, "mode");
    }
  }

  /** The rotor sensor. */
  FeedbackSensor INTERNAL = new Internal();
}
