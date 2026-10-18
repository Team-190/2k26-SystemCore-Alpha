package edu.wpi.team190.gompeilib.core.io.motor.request;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.Hertz;

import org.junit.jupiter.api.Test;

public class RequestOptionsTest {

  @Test
  public void testDefaultMatchesVendorDefaults() {
    RequestOptions options = RequestOptions.DEFAULT;

    assertFalse(options.overrideNeutralMode());
    assertFalse(options.limitForwardMotion());
    assertFalse(options.limitReverseMotion());
    assertFalse(options.ignoreHardwareLimits());
    assertFalse(options.ignoreSoftwareLimits());
    assertFalse(options.useTimesync());
    assertEquals(Hertz.of(100.0), options.updateFrequency());
    assertEquals(RequestOptions.DEFAULT_UPDATE_FREQUENCY, options.updateFrequency());
  }

  @Test
  public void testEveryComponentIsStored() {
    RequestOptions options =
        new RequestOptions(true, false, true, false, true, false, Hertz.of(50));

    assertTrue(options.overrideNeutralMode());
    assertFalse(options.limitForwardMotion());
    assertTrue(options.limitReverseMotion());
    assertFalse(options.ignoreHardwareLimits());
    assertTrue(options.ignoreSoftwareLimits());
    assertFalse(options.useTimesync());
    assertEquals(Hertz.of(50), options.updateFrequency());
  }
}
