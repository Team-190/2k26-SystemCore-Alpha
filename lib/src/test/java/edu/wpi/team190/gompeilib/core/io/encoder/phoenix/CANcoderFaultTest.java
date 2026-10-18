package edu.wpi.team190.gompeilib.core.io.encoder.phoenix;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class CANcoderFaultTest {

  @Test
  public void testMatchesPhoenixFaults() {
    IOTestSupport.assertFaultEnum(CANcoderFault.class, CoreCANcoder.class);
  }
}
