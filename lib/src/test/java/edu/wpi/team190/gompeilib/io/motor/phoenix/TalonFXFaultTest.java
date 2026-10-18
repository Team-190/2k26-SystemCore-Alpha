package edu.wpi.team190.gompeilib.io.motor.phoenix;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import org.junit.jupiter.api.Test;

public class TalonFXFaultTest {

  @Test
  public void testMatchesPhoenixFaults() {
    IOTestSupport.assertFaultEnum(TalonFXFault.class, CoreTalonFX.class);
  }
}
