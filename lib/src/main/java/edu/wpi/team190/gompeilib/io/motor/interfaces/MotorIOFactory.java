package edu.wpi.team190.gompeilib.io.motor.interfaces;

import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;

/** Builds the {@link MotorIO} for one physical motor from its config. */
public interface MotorIOFactory {
  /**
   * Builds a motor.
   *
   * @param config the motor's config
   * @return the motor
   */
  MotorIO create(MotorConfig config);

  /**
   * Returns the controller family this factory builds. Two motors can hardware-follow each other
   * only if their factories report the same family and it supports hardware following.
   *
   * @return the family name
   */
  String family();

  /**
   * Returns whether motors from this family can natively follow a leader of the same family on the
   * same bus.
   *
   * @return true if native following is supported
   */
  default boolean supportsHardwareFollowing() {
    return false;
  }
}
