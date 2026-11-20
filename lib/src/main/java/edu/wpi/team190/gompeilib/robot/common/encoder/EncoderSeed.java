package edu.wpi.team190.gompeilib.robot.common.encoder;

import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.robot.common.core.MechanismCore;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;

/**
 * An absolute encoder on the gearbox output that can seed the motors' position. It is read and
 * logged under {@code <topic>/Encoder} every loop.
 */
public final class EncoderSeed implements MechanismCore.Component {
  private final MechanismCore core;
  private final AbsoluteEncoderIO encoder;
  private final AbsoluteEncoderTelemetryAutoLogged telemetry =
      new AbsoluteEncoderTelemetryAutoLogged();

  /**
   * Creates an encoder seed and registers it with the core.
   *
   * @param core the mechanism's core
   * @param encoder the encoder
   */
  public EncoderSeed(MechanismCore core, AbsoluteEncoderIO encoder) {
    this.core = Objects.requireNonNull(core, "core");
    this.encoder = Objects.requireNonNull(encoder, "encoder");
    core.add(this);
  }

  /**
   * Reads the encoder now, so the seed doesn't wait for the next loop, and resets the motors'
   * position to that reading.
   */
  public void seedPosition() {
    read(core.getTopic());
    core.getMotors().setPosition(telemetry.absolutePosition);
  }

  @Override
  public void periodic(String topic) {
    read(topic);
  }

  private void read(String topic) {
    encoder.updateInputs(telemetry);
    Logger.processInputs(topic + "/Encoder", telemetry);
  }
}
