package edu.wpi.team190.gompeilib.robot.common.encoder;

import edu.wpi.team190.gompeilib.robot.common.core.Mechanism;

/** A mechanism that can seed its motors' position from an absolute encoder. */
public interface EncoderSeedable extends Mechanism {
  /**
   * Returns the mechanism's encoder seed.
   *
   * @return the encoder seed
   */
  EncoderSeed encoderSeed();

  /**
   * Resets the motors' position to the absolute encoder's latest reading. Does nothing without an
   * encoder.
   */
  default void seedPositionFromEncoder() {
    encoderSeed().seedPosition();
  }
}
