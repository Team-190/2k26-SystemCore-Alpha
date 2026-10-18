package edu.wpi.team190.gompeilib.core.io.motor.request;

import static org.wpilib.units.Units.Hertz;

import org.wpilib.units.measure.Frequency;

/**
 * Options shared by most motor requests.
 *
 * @param overrideNeutralMode apply the request's neutral behavior (brake or coast) instead of the
 *     configured neutral mode when the output is zero
 * @param limitForwardMotion prevent forward motion, regardless of limit switches
 * @param limitReverseMotion prevent reverse motion, regardless of limit switches
 * @param ignoreHardwareLimits ignore hardware limit switches
 * @param ignoreSoftwareLimits ignore software (soft) limits
 * @param useTimesync apply the request on the bus's synchronized time base
 * @param updateFrequency how often the request is re-sent
 */
public record RequestOptions(
    boolean overrideNeutralMode,
    boolean limitForwardMotion,
    boolean limitReverseMotion,
    boolean ignoreHardwareLimits,
    boolean ignoreSoftwareLimits,
    boolean useTimesync,
    Frequency updateFrequency) {

  /** The default update frequency for motor requests. */
  public static final Frequency DEFAULT_UPDATE_FREQUENCY = Hertz.of(100.0);

  /** All flags off, re-sent at {@link #DEFAULT_UPDATE_FREQUENCY}. */
  public static final RequestOptions DEFAULT =
      new RequestOptions(false, false, false, false, false, false, DEFAULT_UPDATE_FREQUENCY);
}
