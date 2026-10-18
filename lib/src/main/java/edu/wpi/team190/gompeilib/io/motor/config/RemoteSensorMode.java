package edu.wpi.team190.gompeilib.io.motor.config;

/** How a motor uses a remote absolute encoder for feedback. */
public enum RemoteSensorMode {
  /** The encoder's position is used directly. */
  REMOTE,
  /** The encoder's position is fused with the rotor's for a fast, absolute position. */
  FUSED,
  /** The rotor's position is synchronized to the encoder's at startup, then used alone. */
  SYNC;
}
