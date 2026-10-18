package edu.wpi.team190.gompeilib.core.io.led.phoenix;

import com.ctre.phoenix6.hardware.core.CoreCANdle;
import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class CANdleFaultTest {

  @Test
  public void testMatchesPhoenixFaults() {
    IOTestSupport.assertFaultEnum(CANdleFault.class, CoreCANdle.class);
  }
}
