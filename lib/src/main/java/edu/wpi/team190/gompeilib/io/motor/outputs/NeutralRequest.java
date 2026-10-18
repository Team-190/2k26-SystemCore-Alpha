package edu.wpi.team190.gompeilib.io.motor.outputs;

import org.wpilib.units.measure.Frequency;

/** A request that produces no driving output. */
public sealed interface NeutralRequest extends MotorRequest {
  /**
   * Returns how often the request is re-sent.
   *
   * @return the update frequency
   */
  Frequency updateFrequency();

  /**
   * Applies the configured neutral mode (brake or coast).
   *
   * @param useTimesync apply the request on the bus's synchronized time base
   * @param updateFrequency how often the request is re-sent
   */
  record Neutral(boolean useTimesync, Frequency updateFrequency) implements NeutralRequest {
    public Neutral() {
      this(false, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    }
  }

  /**
   * Coasts, regardless of the configured neutral mode.
   *
   * @param useTimesync apply the request on the bus's synchronized time base
   * @param updateFrequency how often the request is re-sent
   */
  record Coast(boolean useTimesync, Frequency updateFrequency) implements NeutralRequest {
    public Coast() {
      this(false, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    }
  }

  /**
   * Brakes by shorting the motor leads, regardless of the configured neutral mode.
   *
   * @param useTimesync apply the request on the bus's synchronized time base
   * @param updateFrequency how often the request is re-sent
   */
  record StaticBrake(boolean useTimesync, Frequency updateFrequency) implements NeutralRequest {
    public StaticBrake() {
      this(false, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    }
  }

  /**
   * Plays an audible tone through the motor.
   *
   * @param audioFrequency the tone's pitch
   * @param updateFrequency how often the request is re-sent
   */
  record Tone(Frequency audioFrequency, Frequency updateFrequency) implements NeutralRequest {
    public Tone(Frequency audioFrequency) {
      this(audioFrequency, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    }
  }
}
