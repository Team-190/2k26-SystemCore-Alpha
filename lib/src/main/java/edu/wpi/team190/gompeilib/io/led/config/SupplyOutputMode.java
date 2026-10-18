package edu.wpi.team190.gompeilib.io.led.config;

/** What an LED controller's supply-voltage output does. */
public enum SupplyOutputMode {
  /** Always on. */
  ON,
  /** Always off. */
  OFF,
  /** Modulated by supply-modulation requests. */
  MODULATED;
}
