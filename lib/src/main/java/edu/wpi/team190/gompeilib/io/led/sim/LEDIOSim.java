package edu.wpi.team190.gompeilib.io.led.sim;

import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.io.led.inputs.LEDTelemetry;
import edu.wpi.team190.gompeilib.io.led.interfaces.LEDIO;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDAnimation;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDColor;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRequest;
import java.util.Arrays;
import java.util.Optional;
import org.wpilib.system.RobotController;

/**
 * A simulated LED controller. It keeps each LED's solid color and which animation each slot is
 * running, so tests and visualizations can read them back; it doesn't render animations.
 */
public class LEDIOSim implements LEDIO {
  /** How many animations can run at once, matching the CANdle's slot count. */
  public static final int ANIMATION_SLOTS = 8;

  private static final LEDColor OFF = new LEDColor(0, 0, 0, 0);

  private final LEDColor[] colors;
  private final LEDAnimation[] animations = new LEDAnimation[ANIMATION_SLOTS];
  private double supplyModulation = 0.0;

  /**
   * Creates a simulated LED controller.
   *
   * @param ledCount the number of LEDs on the strip, including any on the controller itself
   */
  public LEDIOSim(int ledCount) {
    colors = new LEDColor[ledCount];
    Arrays.fill(colors, OFF);
  }

  @Override
  public boolean isConnected() {
    return true;
  }

  @Override
  public void setControl(LEDRequest request) {
    switch (request) {
      case LEDRequest.Solid solid -> {
        int end = Math.min(solid.range().end(), colors.length - 1);
        for (int i = solid.range().start(); i <= end; i++) {
          colors[i] = solid.color();
        }
      }
      case LEDRequest.SupplyModulation modulation -> supplyModulation = modulation.output();
      case LEDAnimation.ClearAnimation clear -> animations[slot(clear)] = null;
      case LEDAnimation animation -> animations[slot(animation)] = animation;
    }
  }

  private static int slot(LEDAnimation animation) {
    if (animation.slot() < 0 || animation.slot() >= ANIMATION_SLOTS) {
      throw new IllegalArgumentException(
          "animation slot must be in [0, " + (ANIMATION_SLOTS - 1) + "], got " + animation.slot());
    }
    return animation.slot();
  }

  /**
   * Returns an LED's solid color.
   *
   * @param index the LED's index
   * @return the color last set by a solid request, or off
   */
  public LEDColor color(int index) {
    return colors[index];
  }

  /**
   * Returns the animation running in a slot.
   *
   * @param slot the slot
   * @return the animation, or empty if the slot is clear
   */
  public Optional<LEDAnimation> animation(int slot) {
    return Optional.ofNullable(animations[slot]);
  }

  @Override
  public void updateInputs(LEDTelemetry telemetry) {
    telemetry.connected = true;
    telemetry.supplyVoltage = Volts.of(RobotController.getBatteryVoltage());
    telemetry.fiveVoltRailVoltage = Volts.of(5.0);
    telemetry.supplyModulation = supplyModulation;
    telemetry.maxSimultaneousAnimations = ANIMATION_SLOTS;
  }
}
