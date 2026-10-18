package edu.wpi.team190.gompeilib.core.io.motor.request;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class OpenLoopRequestTest {

  @Test
  public void testCanonicalConstructorsStoreEveryComponent() throws ReflectiveOperationException {
    IOTestSupport.assertCanonicalConstructors(OpenLoopRequest.class);
  }

  @Test
  public void testShortConstructorsUseDefaults() throws ReflectiveOperationException {
    IOTestSupport.assertShortConstructors(
        OpenLoopRequest.class, IOTestSupport.MOTOR_REQUEST_DEFAULTS);
  }

  @Test
  public void testOutputTypeMatchesName() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(OpenLoopRequest.class)) {
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
}
