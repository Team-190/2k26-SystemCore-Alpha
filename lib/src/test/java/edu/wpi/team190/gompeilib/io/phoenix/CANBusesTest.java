package edu.wpi.team190.gompeilib.io.phoenix;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix6.CANBus;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;

public class CANBusesTest {

  @Test
  public void testToCANBus() {
    assertEquals(new CANBus(CANPort.CAN_S1), CANBuses.toCANBus(CANBusId.port(CANPort.CAN_S1)));
    assertEquals(new CANBus("Drive"), CANBuses.toCANBus(CANBusId.named("Drive")));
  }

  @Test
  public void testToCANBusIdRoundTripsNamedBuses() {
    assertEquals(CANBusId.named("Drive"), CANBuses.toCANBusId(new CANBus("Drive")));
  }
}
