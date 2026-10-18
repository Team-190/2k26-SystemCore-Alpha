package edu.wpi.team190.gompeilib.io.gyro.phoenix;

import com.ctre.phoenix6.hardware.core.CorePigeon2;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class Pigeon2FaultTest {

  @Test
  public void testMatchesPhoenixFaults() {
    IOTestSupport.assertFaultEnum(Pigeon2Fault.class, CorePigeon2.class);
  }
}
