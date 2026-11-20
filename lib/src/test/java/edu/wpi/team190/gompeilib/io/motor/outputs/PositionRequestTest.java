package edu.wpi.team190.gompeilib.io.motor.outputs;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class PositionRequestTest {

  @Test
  public void testCanonicalConstructorsStoreEveryComponent() throws ReflectiveOperationException {
    IOTestSupport.assertCanonicalConstructors(PositionRequest.class);
  }

  @Test
  public void testShortConstructorsUseDefaults() throws ReflectiveOperationException {
    IOTestSupport.assertShortConstructors(
        PositionRequest.class, IOTestSupport.MOTOR_REQUEST_DEFAULTS);
  }

  @Test
  public void testOutputTypeMatchesName() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(PositionRequest.class)) {
      AxisRequest request = (AxisRequest) IOTestSupport.sampleRecord(record);
      String name = record.getSimpleName();
      OutputType expected =
          name.endsWith("DutyCycle") || name.equals("DutyCycleOutput")
              ? OutputType.DUTY_CYCLE
              : name.endsWith("Voltage") || name.equals("VoltageOutput")
                  ? OutputType.VOLTAGE
                  : OutputType.CURRENT;
      assertEquals(expected, request.outputType(), name);
      assertNotNull(request.options(), name);
    }
  }

  @Test
  public void testPositionAccessorIsTheFirstComponent() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(PositionRequest.class)) {
      PositionRequest request = (PositionRequest) IOTestSupport.sampleRecord(record);
      assertEquals(
          IOTestSupport.sample(org.wpilib.units.measure.Angle.class, 1), request.position());
    }
  }

  @Test
  public void testOfPicksRequestFromModeAndFeedforward() {
    org.wpilib.units.measure.Angle position = org.wpilib.units.Units.Radians.of(1.0);
    edu.wpi.team190.gompeilib.io.motor.config.GainSlot slot =
        edu.wpi.team190.gompeilib.io.motor.config.GainSlot.ONE;
    Feedforward volts = Feedforward.volts(org.wpilib.units.Units.Volts.of(0.5));
    Feedforward amps = Feedforward.amps(org.wpilib.units.Units.Amps.of(2.0));
    Feedforward duty = Feedforward.dutyCycle(0.1);
    RequestOptions options = RequestOptions.DEFAULT;

    assertInstanceOf(
        PositionRequest.ProfiledPositionVoltage.class,
        PositionRequest.of(ProfileMode.PROFILED, position, volts, true, slot, options));
    assertInstanceOf(
        PositionRequest.PositionVoltage.class,
        PositionRequest.of(ProfileMode.UNPROFILED, position, volts, true, slot, options));
    assertInstanceOf(
        PositionRequest.ExponentialProfiledPositionVoltage.class,
        PositionRequest.of(ProfileMode.EXPONENTIAL, position, volts, true, slot, options));
    assertInstanceOf(
        PositionRequest.ProfiledPositionCurrent.class,
        PositionRequest.of(ProfileMode.PROFILED, position, amps, true, slot, options));
    assertInstanceOf(
        PositionRequest.PositionCurrent.class,
        PositionRequest.of(ProfileMode.UNPROFILED, position, amps, true, slot, options));
    assertInstanceOf(
        PositionRequest.ExponentialProfiledPositionCurrent.class,
        PositionRequest.of(ProfileMode.EXPONENTIAL, position, amps, true, slot, options));
    assertInstanceOf(
        PositionRequest.ProfiledPositionDutyCycle.class,
        PositionRequest.of(ProfileMode.PROFILED, position, duty, true, slot, options));
    assertInstanceOf(
        PositionRequest.PositionDutyCycle.class,
        PositionRequest.of(ProfileMode.UNPROFILED, position, duty, true, slot, options));
    assertInstanceOf(
        PositionRequest.ExponentialProfiledPositionDutyCycle.class,
        PositionRequest.of(ProfileMode.EXPONENTIAL, position, duty, true, slot, options));
  }

  @Test
  public void testOfCarriesItsArguments() {
    RequestOptions options =
        new RequestOptions(
            true, false, false, false, true, false, RequestOptions.DEFAULT_UPDATE_FREQUENCY);
    PositionRequest.PositionVoltage request =
        (PositionRequest.PositionVoltage)
            PositionRequest.of(
                ProfileMode.UNPROFILED,
                org.wpilib.units.Units.Radians.of(2.0),
                Feedforward.volts(org.wpilib.units.Units.Volts.of(1.5)),
                false,
                edu.wpi.team190.gompeilib.io.motor.config.GainSlot.TWO,
                options);
    assertEquals(org.wpilib.units.Units.Radians.of(2.0), request.position());
    assertEquals(org.wpilib.units.Units.RadiansPerSecond.of(0.0), request.velocity());
    assertEquals(org.wpilib.units.Units.Volts.of(1.5), request.feedforward());
    assertFalse(request.foc());
    assertEquals(edu.wpi.team190.gompeilib.io.motor.config.GainSlot.TWO, request.slot());
    assertEquals(options, request.options());
  }
}
