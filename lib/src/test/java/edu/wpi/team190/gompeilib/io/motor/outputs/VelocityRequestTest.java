package edu.wpi.team190.gompeilib.io.motor.outputs;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class VelocityRequestTest {

  @Test
  public void testCanonicalConstructorsStoreEveryComponent() throws ReflectiveOperationException {
    IOTestSupport.assertCanonicalConstructors(VelocityRequest.class);
  }

  @Test
  public void testShortConstructorsUseDefaults() throws ReflectiveOperationException {
    IOTestSupport.assertShortConstructors(
        VelocityRequest.class, IOTestSupport.MOTOR_REQUEST_DEFAULTS);
  }

  @Test
  public void testOutputTypeMatchesName() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(VelocityRequest.class)) {
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
  public void testVelocityAccessorIsTheFirstComponent() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(VelocityRequest.class)) {
      VelocityRequest request = (VelocityRequest) IOTestSupport.sampleRecord(record);
      assertEquals(
          IOTestSupport.sample(org.wpilib.units.measure.AngularVelocity.class, 1),
          request.velocity());
    }
  }

  @Test
  public void testOfPicksRequestFromModeAndFeedforward() {
    org.wpilib.units.measure.AngularVelocity velocity =
        org.wpilib.units.Units.RadiansPerSecond.of(10.0);
    org.wpilib.units.measure.AngularAcceleration acceleration =
        org.wpilib.units.Units.RadiansPerSecondPerSecond.of(0.0);
    edu.wpi.team190.gompeilib.io.motor.config.GainSlot slot =
        edu.wpi.team190.gompeilib.io.motor.config.GainSlot.ONE;
    RequestOptions options = RequestOptions.DEFAULT;
    Feedforward volts = Feedforward.volts(org.wpilib.units.Units.Volts.of(0.5));
    Feedforward amps = Feedforward.amps(org.wpilib.units.Units.Amps.of(2.0));
    Feedforward duty = Feedforward.dutyCycle(0.1);

    assertInstanceOf(
        VelocityRequest.ProfiledVelocityVoltage.class,
        VelocityRequest.of(
            VelocityProfileMode.PROFILED, velocity, acceleration, volts, true, slot, options));
    assertInstanceOf(
        VelocityRequest.VelocityVoltage.class,
        VelocityRequest.of(
            VelocityProfileMode.UNPROFILED, velocity, acceleration, volts, true, slot, options));
    assertInstanceOf(
        VelocityRequest.ProfiledVelocityCurrent.class,
        VelocityRequest.of(
            VelocityProfileMode.PROFILED, velocity, acceleration, amps, true, slot, options));
    assertInstanceOf(
        VelocityRequest.VelocityCurrent.class,
        VelocityRequest.of(
            VelocityProfileMode.UNPROFILED, velocity, acceleration, amps, true, slot, options));
    assertInstanceOf(
        VelocityRequest.ProfiledVelocityDutyCycle.class,
        VelocityRequest.of(
            VelocityProfileMode.PROFILED, velocity, acceleration, duty, true, slot, options));
    assertInstanceOf(
        VelocityRequest.VelocityDutyCycle.class,
        VelocityRequest.of(
            VelocityProfileMode.UNPROFILED, velocity, acceleration, duty, true, slot, options));
  }
}
