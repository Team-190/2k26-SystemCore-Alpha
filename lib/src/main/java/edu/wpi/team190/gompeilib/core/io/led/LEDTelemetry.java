package edu.wpi.team190.gompeilib.core.io.led;

import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.*;

/** Everything an LED controller reports. Fields a device doesn't report stay at their defaults. */
@AutoLog
public class LEDTelemetry {
  /** Whether the device is connected and communicating. */
  public boolean connected = false;

  /** Whether the device has a Pro license. */
  public boolean isProLicensed = false;

  /** The device's packed firmware version. */
  public int firmwareVersion = 0;

  /** Voltage supplied to the device. */
  public Voltage supplyVoltage = Volts.of(0.0);

  /** Voltage on the device's 5 V rail. */
  public Voltage fiveVoltRailVoltage = Volts.of(0.0);

  /** Current drawn by the LEDs. */
  public Current outputCurrent = Amps.of(0.0);

  /** Device temperature. */
  public Temperature deviceTemperature = Celsius.of(0.0);

  /** Applied supply-voltage modulation duty cycle, in [0, 1]. */
  public double supplyModulation = 0.0;

  /** How many animations the device can run at once. */
  public int maxSimultaneousAnimations = 0;

  /** Live faults, packed per the vendor's fault enum. */
  public long faults = 0L;

  /** Sticky faults, packed per the vendor's fault enum. */
  public long stickyFaults = 0L;
}
