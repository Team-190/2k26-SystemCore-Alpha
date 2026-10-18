package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.wpilib.hardware.bus.CANPort;

public class CANBusIdTest {

  @Test
  public void testPortFactory() {
    CANBusId bus = CANBusId.port(CANPort.CAN_S2);

    assertEquals(new CANBusId.Port(CANPort.CAN_S2), bus);
    assertEquals(CANPort.CAN_S2, ((CANBusId.Port) bus).port());
  }

  @Test
  public void testNamedFactory() {
    CANBusId bus = CANBusId.named("Drive");

    assertEquals(new CANBusId.Named("Drive"), bus);
    assertEquals("Drive", ((CANBusId.Named) bus).name());
  }

  @Test
  public void testBusesCompareByValue() {
    assertEquals(CANBusId.port(CANPort.CAN_S1), CANBusId.port(CANPort.CAN_S1));
    assertNotEquals(CANBusId.port(CANPort.CAN_S1), CANBusId.port(CANPort.CAN_S2));
    assertNotEquals(CANBusId.named("CAN_S1"), CANBusId.port(CANPort.CAN_S1));
  }

  @Test
  public void shouldRejectNulls() {
    assertThrows(NullPointerException.class, () -> CANBusId.port(null));
    assertThrows(NullPointerException.class, () -> CANBusId.named(null));
  }
}
