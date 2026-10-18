package edu.wpi.team190.gompeilib.core.io.range.phoenix;

import com.ctre.phoenix6.hardware.core.CoreCANrange;
import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class CANrangeFaultTest {

  @Test
  public void testMatchesPhoenixFaults() {
    IOTestSupport.assertFaultEnum(CANrangeFault.class, CoreCANrange.class);
  }
}
