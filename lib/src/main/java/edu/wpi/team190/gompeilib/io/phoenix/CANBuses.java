package edu.wpi.team190.gompeilib.io.phoenix;

import com.ctre.phoenix6.CANBus;
import edu.wpi.team190.gompeilib.io.base.CANBusId;

/** Converts GompeiLib's vendor-neutral bus identifiers to Phoenix6 buses. */
public final class CANBuses {
  private CANBuses() {}

  /**
   * Converts a bus identifier.
   *
   * @param bus the bus
   * @return the Phoenix6 bus
   */
  public static CANBus toCANBus(CANBusId bus) {
    return switch (bus) {
      case CANBusId.Port port -> new CANBus(port.port());
      case CANBusId.Named named -> new CANBus(named.name());
    };
  }

  /**
   * Converts a Phoenix6 bus back to an identifier, by name.
   *
   * @param bus the Phoenix6 bus
   * @return the bus identifier
   */
  public static CANBusId toCANBusId(CANBus bus) {
    return CANBusId.named(bus.getName());
  }
}
