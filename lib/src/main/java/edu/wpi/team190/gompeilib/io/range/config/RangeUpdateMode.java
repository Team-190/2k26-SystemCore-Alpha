package edu.wpi.team190.gompeilib.io.range.config;

/** How a range sensor trades range for update rate. */
public enum RangeUpdateMode {
  /** Short range, updating at 100 Hz. */
  SHORT_RANGE_100HZ,
  /** Short range, updating at the configured frequency. */
  SHORT_RANGE_USER_FREQUENCY,
  /** Long range, updating at the configured frequency. */
  LONG_RANGE_USER_FREQUENCY;
}
