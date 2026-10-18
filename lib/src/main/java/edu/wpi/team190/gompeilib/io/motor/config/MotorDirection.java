package edu.wpi.team190.gompeilib.io.motor.config;

/** Which rotor direction a motor treats as positive output. */
public enum MotorDirection {
  COUNTER_CLOCKWISE_POSITIVE,
  CLOCKWISE_POSITIVE;

  /**
   * Returns the opposite direction.
   *
   * @return the other direction
   */
  public MotorDirection opposite() {
    return this == COUNTER_CLOCKWISE_POSITIVE ? CLOCKWISE_POSITIVE : COUNTER_CLOCKWISE_POSITIVE;
  }
}
