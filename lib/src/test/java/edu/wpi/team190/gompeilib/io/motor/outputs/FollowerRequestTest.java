package edu.wpi.team190.gompeilib.io.motor.outputs;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class FollowerRequestTest {

  @Test
  public void testCanonicalConstructorsStoreEveryComponent() throws ReflectiveOperationException {
    IOTestSupport.assertCanonicalConstructors(FollowerRequest.class);
  }

  @Test
  public void testShortConstructorsUseDefaults() throws ReflectiveOperationException {
    IOTestSupport.assertShortConstructors(
        FollowerRequest.class, IOTestSupport.MOTOR_REQUEST_DEFAULTS);
  }

  @Test
  public void testSharedAccessors() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(FollowerRequest.class)) {
      FollowerRequest request = (FollowerRequest) IOTestSupport.sampleRecord(record);
      assertEquals(1, request.leaderId(), record.getSimpleName());
      assertNotNull(request.updateFrequency(), record.getSimpleName());
    }
  }
}
