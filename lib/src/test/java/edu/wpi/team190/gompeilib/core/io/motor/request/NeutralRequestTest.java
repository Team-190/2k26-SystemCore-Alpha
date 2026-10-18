package edu.wpi.team190.gompeilib.core.io.motor.request;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class NeutralRequestTest {

  @Test
  public void testCanonicalConstructorsStoreEveryComponent() throws ReflectiveOperationException {
    IOTestSupport.assertCanonicalConstructors(NeutralRequest.class);
  }

  @Test
  public void testShortConstructorsUseDefaults() throws ReflectiveOperationException {
    IOTestSupport.assertShortConstructors(
        NeutralRequest.class, IOTestSupport.MOTOR_REQUEST_DEFAULTS);
  }

  @Test
  public void testSharedAccessors() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(NeutralRequest.class)) {
      NeutralRequest request = (NeutralRequest) IOTestSupport.sampleRecord(record);
      assertNotNull(request.updateFrequency(), record.getSimpleName());
    }
  }
}
