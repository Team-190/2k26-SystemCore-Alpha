package edu.wpi.team190.gompeilib.robot.mechanisms;

import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import java.util.Objects;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * An optional absolute encoder on the gearbox output that can seed the motors' position. It is read
 * and logged under {@code <topic>/Encoder} every loop.
 */
public final class EncoderSeed implements MechanismCore.Component {
  private final MechanismCore core;
  private final Optional<AbsoluteEncoderIO> encoder;
  private final AbsoluteEncoderTelemetryAutoLogged telemetry =
      new AbsoluteEncoderTelemetryAutoLogged();

  /**
   * Creates an encoder seed and registers it with the core.
   *
   * @param core the mechanism's core
   * @param encoder the encoder, or empty for none
   */
  public EncoderSeed(MechanismCore core, Optional<AbsoluteEncoderIO> encoder) {
    this.core = core;
    this.encoder = Objects.requireNonNull(encoder, "encoder");
    core.add(this);
  }

  /**
   * Returns whether an encoder is attached.
   *
   * @return true if there is one
   */
  public boolean isPresent() {
    return encoder.isPresent();
  }

  /**
   * Resets the motors' position to the encoder's latest reading. Does nothing without an encoder.
   */
  public void seedPosition() {
    encoder.ifPresent(io -> core.getMotors().setPosition(telemetry.absolutePosition));
  }

  @Override
  public void periodic(String topic) {
    encoder.ifPresent(
        io -> {
          io.updateInputs(telemetry);
          Logger.processInputs(topic + "/Encoder", telemetry);
        });
  }
}
