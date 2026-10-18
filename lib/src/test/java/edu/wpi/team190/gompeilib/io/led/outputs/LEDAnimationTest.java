package edu.wpi.team190.gompeilib.io.led.outputs;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class LEDAnimationTest {

  @Test
  public void testCanonicalConstructorsStoreEveryComponent() throws ReflectiveOperationException {
    IOTestSupport.assertCanonicalConstructors(LEDAnimation.class);
  }

  @Test
  public void testSharedAccessors() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(LEDAnimation.class)) {
      LEDAnimation animation = (LEDAnimation) IOTestSupport.sampleRecord(record);
      assertTrue(animation.slot() > 0, record.getSimpleName());
      assertNotNull(animation.updateFrequency(), record.getSimpleName());
    }
  }
}
