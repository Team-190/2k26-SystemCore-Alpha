package edu.wpi.team190.gompeilib.io.led.phoenix;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.team190.gompeilib.io.led.config.LEDConfig;
import edu.wpi.team190.gompeilib.io.led.outputs.AnimationDirection;
import edu.wpi.team190.gompeilib.io.led.outputs.BounceMode;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDAnimation;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDColor;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRequest;

/** Pure conversions between GompeiLib's LED types and Phoenix6's CANdle types. */
public final class CANdleConversions {
  private CANdleConversions() {}

  /**
   * Builds the device configuration for an LED controller.
   *
   * @param config the controller's config
   * @return the CANdle configuration
   */
  public static CANdleConfiguration toConfiguration(LEDConfig config) {
    CANdleConfiguration candle = new CANdleConfiguration();
    candle.LED.StripType = StripTypeValue.valueOf(config.stripType.name());
    candle.LED.BrightnessScalar = config.brightness;
    candle.LED.LossOfSignalBehavior =
        config.keepRunningOnLossOfSignal
            ? LossOfSignalBehaviorValue.KeepRunning
            : LossOfSignalBehaviorValue.DisableLEDs;
    candle.CANdleFeatures.Enable5VRail =
        config.enableFiveVoltRail ? Enable5VRailValue.Enabled : Enable5VRailValue.Disabled;
    candle.CANdleFeatures.VBatOutputMode =
        switch (config.supplyOutputMode) {
          case ON -> VBatOutputModeValue.On;
          case OFF -> VBatOutputModeValue.Off;
          case MODULATED -> VBatOutputModeValue.Modulated;
        };
    candle.CANdleFeatures.StatusLedWhenActive =
        config.statusLedWhenActive
            ? StatusLedWhenActiveValue.Enabled
            : StatusLedWhenActiveValue.Disabled;
    return candle;
  }

  /**
   * Converts an LED request to the Phoenix6 request that runs it.
   *
   * @param request the request
   * @return the Phoenix6 request
   */
  public static ControlRequest toControlRequest(LEDRequest request) {
    return switch (request) {
      case LEDRequest.Solid r ->
          new SolidColor(r.range().start(), r.range().end()).withColor(toColor(r.color()));
      case LEDRequest.SupplyModulation r ->
          new ModulateVBatOut(r.output()).withUpdateFreqHz(r.updateFrequency());
      case LEDAnimation.ColorFlow r ->
          new ColorFlowAnimation(r.range().start(), r.range().end())
              .withSlot(r.slot())
              .withColor(toColor(r.color()))
              .withDirection(toDirection(r.direction()))
              .withFrameRate(r.frameRate())
              .withUpdateFreqHz(r.updateFrequency());
      case LEDAnimation.Fire r ->
          new FireAnimation(r.range().start(), r.range().end())
              .withSlot(r.slot())
              .withBrightness(r.brightness())
              .withDirection(toDirection(r.direction()))
              .withSparking(r.sparking())
              .withCooling(r.cooling())
              .withFrameRate(r.frameRate())
              .withUpdateFreqHz(r.updateFrequency());
      case LEDAnimation.Larson r ->
          new LarsonAnimation(r.range().start(), r.range().end())
              .withSlot(r.slot())
              .withColor(toColor(r.color()))
              .withSize(r.size())
              .withBounceMode(toBounce(r.bounce()))
              .withFrameRate(r.frameRate())
              .withUpdateFreqHz(r.updateFrequency());
      case LEDAnimation.Rainbow r ->
          new RainbowAnimation(r.range().start(), r.range().end())
              .withSlot(r.slot())
              .withBrightness(r.brightness())
              .withDirection(toDirection(r.direction()))
              .withFrameRate(r.frameRate())
              .withUpdateFreqHz(r.updateFrequency());
      case LEDAnimation.RgbFade r ->
          new RgbFadeAnimation(r.range().start(), r.range().end())
              .withSlot(r.slot())
              .withBrightness(r.brightness())
              .withFrameRate(r.frameRate())
              .withUpdateFreqHz(r.updateFrequency());
      case LEDAnimation.SingleFade r ->
          new SingleFadeAnimation(r.range().start(), r.range().end())
              .withSlot(r.slot())
              .withColor(toColor(r.color()))
              .withFrameRate(r.frameRate())
              .withUpdateFreqHz(r.updateFrequency());
      case LEDAnimation.Strobe r ->
          new StrobeAnimation(r.range().start(), r.range().end())
              .withSlot(r.slot())
              .withColor(toColor(r.color()))
              .withFrameRate(r.frameRate())
              .withUpdateFreqHz(r.updateFrequency());
      case LEDAnimation.Twinkle r ->
          new TwinkleAnimation(r.range().start(), r.range().end())
              .withSlot(r.slot())
              .withColor(toColor(r.color()))
              .withMaxLEDsOnProportion(r.maxOnProportion())
              .withFrameRate(r.frameRate())
              .withUpdateFreqHz(r.updateFrequency());
      case LEDAnimation.TwinkleOff r ->
          new TwinkleOffAnimation(r.range().start(), r.range().end())
              .withSlot(r.slot())
              .withColor(toColor(r.color()))
              .withMaxLEDsOnProportion(r.maxOnProportion())
              .withFrameRate(r.frameRate())
              .withUpdateFreqHz(r.updateFrequency());
      case LEDAnimation.ClearAnimation r ->
          new EmptyAnimation(r.slot()).withUpdateFreqHz(r.updateFrequency());
    };
  }

  /**
   * Converts a color.
   *
   * @param color the color
   * @return the Phoenix6 color
   */
  public static RGBWColor toColor(LEDColor color) {
    return new RGBWColor(color.red(), color.green(), color.blue(), color.white());
  }

  /**
   * Converts an animation direction.
   *
   * @param direction the direction
   * @return the Phoenix6 direction
   */
  public static AnimationDirectionValue toDirection(AnimationDirection direction) {
    return direction == AnimationDirection.FORWARD
        ? AnimationDirectionValue.Forward
        : AnimationDirectionValue.Backward;
  }

  /**
   * Converts a bounce mode.
   *
   * @param bounce the bounce mode
   * @return the Phoenix6 bounce mode
   */
  public static LarsonBounceValue toBounce(BounceMode bounce) {
    return switch (bounce) {
      case FRONT -> LarsonBounceValue.Front;
      case CENTER -> LarsonBounceValue.Center;
      case BACK -> LarsonBounceValue.Back;
    };
  }
}
