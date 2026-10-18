package edu.wpi.team190.gompeilib.io.base;

import java.util.Objects;
import org.wpilib.hardware.bus.CANPort;

/** A vendor-neutral CAN bus identifier: either a SystemCore CAN port or a named bus. */
public sealed interface CANBusId {

  /**
   * A bus on one of SystemCore's built-in CAN ports.
   *
   * @param port the port
   */
  record Port(CANPort port) implements CANBusId {
    public Port {
      Objects.requireNonNull(port, "port");
    }
  }

  /**
   * A bus identified by name, such as a CAN FD adapter.
   *
   * @param name the bus name
   */
  record Named(String name) implements CANBusId {
    public Named {
      Objects.requireNonNull(name, "name");
    }
  }

  /**
   * Returns the bus on a SystemCore CAN port.
   *
   * @param port the port
   * @return the bus identifier
   */
  static CANBusId port(CANPort port) {
    return new Port(port);
  }

  /**
   * Returns the bus with the given name.
   *
   * @param name the bus name
   * @return the bus identifier
   */
  static CANBusId named(String name) {
    return new Named(name);
  }
}
