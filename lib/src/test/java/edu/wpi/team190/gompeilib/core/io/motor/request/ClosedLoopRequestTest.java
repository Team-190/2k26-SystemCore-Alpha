package edu.wpi.team190.gompeilib.core.io.motor.request;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class ClosedLoopRequestTest {

  @Test
  public void testCanonicalConstructorsStoreEveryComponent() throws ReflectiveOperationException {
    IOTestSupport.assertCanonicalConstructors(ClosedLoopRequest.class);
  }

  @Test
  public void testShortConstructorsUseDefaults() throws ReflectiveOperationException {
    IOTestSupport.assertShortConstructors(
        ClosedLoopRequest.class, IOTestSupport.MOTOR_REQUEST_DEFAULTS);
  }

  @Test
  public void testOutputTypeMatchesName() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(ClosedLoopRequest.class)) {
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
  public void testSlotAccessor() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(ClosedLoopRequest.class)) {
      ClosedLoopRequest request = (ClosedLoopRequest) IOTestSupport.sampleRecord(record);
      assertNotNull(request.slot(), record.getSimpleName());
    }
  }
}
