package edu.wpi.team190.gompeilib.io.led.phoenix;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.led.config.LEDConfig;
import edu.wpi.team190.gompeilib.io.led.config.LEDStripType;
import edu.wpi.team190.gompeilib.io.led.config.SupplyOutputMode;
import edu.wpi.team190.gompeilib.io.led.outputs.AnimationDirection;
import edu.wpi.team190.gompeilib.io.led.outputs.BounceMode;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDColor;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRange;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRequest;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRequestTest;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class CANdleConversionsTest {

  private static LEDConfig.LEDConfigBuilder config() {
    return LEDConfig.builder().withCanId(1).withBus(CANBusId.named("rio"));
  }

  @Test
  public void testToConfiguration() {
    CANdleConfiguration candle =
        CANdleConversions.toConfiguration(
            config()
                .withStripType(LEDStripType.BRGW)
                .withBrightness(0.25)
                .withKeepRunningOnLossOfSignal(false)
                .withEnableFiveVoltRail(false)
                .withSupplyOutputMode(SupplyOutputMode.OFF)
                .withStatusLedWhenActive(false)
                .build());

    assertEquals(StripTypeValue.BRGW, candle.LED.StripType);
    assertEquals(0.25, candle.LED.BrightnessScalar, 1e-9);
    assertEquals(LossOfSignalBehaviorValue.DisableLEDs, candle.LED.LossOfSignalBehavior);
    assertEquals(Enable5VRailValue.Disabled, candle.CANdleFeatures.Enable5VRail);
    assertEquals(VBatOutputModeValue.Off, candle.CANdleFeatures.VBatOutputMode);
    assertEquals(StatusLedWhenActiveValue.Disabled, candle.CANdleFeatures.StatusLedWhenActive);

    assertEquals(
        VBatOutputModeValue.Modulated,
        CANdleConversions.toConfiguration(
                config().withSupplyOutputMode(SupplyOutputMode.MODULATED).build())
            .CANdleFeatures
            .VBatOutputMode);
  }

  @Test
  public void testDefaultConfigurationMatchesTheDevice() {
    CANdleConfiguration converted = CANdleConversions.toConfiguration(config().build());
    CANdleConfiguration factory = new CANdleConfiguration();

    assertEquals(factory.LED.StripType, converted.LED.StripType);
    assertEquals(factory.LED.BrightnessScalar, converted.LED.BrightnessScalar);
    assertEquals(factory.LED.LossOfSignalBehavior, converted.LED.LossOfSignalBehavior);
    assertEquals(factory.CANdleFeatures.Enable5VRail, converted.CANdleFeatures.Enable5VRail);
    assertEquals(factory.CANdleFeatures.VBatOutputMode, converted.CANdleFeatures.VBatOutputMode);
    assertEquals(
        factory.CANdleFeatures.StatusLedWhenActive, converted.CANdleFeatures.StatusLedWhenActive);
  }

  @Test
  public void testEveryStripTypeMaps() {
    for (LEDStripType type : LEDStripType.values()) {
      assertEquals(
          type.name(),
          CANdleConversions.toConfiguration(config().withStripType(type).build())
              .LED
              .StripType
              .name());
    }
  }

  @Test
  public void testEveryRequestMapsToItsPhoenixRequest() throws ReflectiveOperationException {
    Map<Class<?>, String> phoenixNames = new HashMap<>();
    LEDRequestTest.CANDLE_REQUESTS.forEach((phoenix, ours) -> phoenixNames.put(ours, phoenix));

    for (Class<?> type : IOTestSupport.leafRecords(LEDRequest.class)) {
      ControlRequest phoenix =
          CANdleConversions.toControlRequest((LEDRequest) IOTestSupport.sampleRecord(type));
      assertEquals(
          phoenixNames.get(type), phoenix.getClass().getSimpleName(), type.getSimpleName());
    }
  }

  /** Phoenix6 CANdle request field name to {@code LEDRequest} record component name. */
  private static final Map<String, String> LED_FIELD_NAMES =
      Map.ofEntries(
          Map.entry("LEDStartIndex", "range"),
          Map.entry("LEDEndIndex", "range"),
          Map.entry("Slot", "slot"),
          Map.entry("Color", "color"),
          Map.entry("Direction", "direction"),
          Map.entry("Brightness", "brightness"),
          Map.entry("Sparking", "sparking"),
          Map.entry("Cooling", "cooling"),
          Map.entry("Size", "size"),
          Map.entry("BounceMode", "bounce"),
          Map.entry("MaxLEDsOnProportion", "maxOnProportion"),
          Map.entry("FrameRate", "frameRate"),
          Map.entry("Output", "output"),
          Map.entry("UpdateFreqHz", "updateFrequency"));

  @Test
  public void testEveryRequestCarriesEveryPhoenixSetting() throws ClassNotFoundException {
    for (Map.Entry<String, Class<? extends LEDRequest>> mapping :
        LEDRequestTest.CANDLE_REQUESTS.entrySet()) {
      IOTestSupport.assertFieldsCovered(
          Class.forName("com.ctre.phoenix6.controls." + mapping.getKey()),
          mapping.getValue(),
          LED_FIELD_NAMES,
          java.util.Set.of());
    }
  }

  @Test
  public void testSolidColorCarriesRangeAndColor() {
    SolidColor solid =
        (SolidColor)
            CANdleConversions.toControlRequest(
                new LEDRequest.Solid(new LEDRange(2, 9), new LEDColor(1, 2, 3, 4)));

    assertEquals(2, solid.LEDStartIndex);
    assertEquals(9, solid.LEDEndIndex);
    assertEquals(1, solid.Color.Red);
    assertEquals(4, solid.Color.White);
  }

  @Test
  public void testValueConversions() {
    RGBWColor color = CANdleConversions.toColor(new LEDColor(10, 20, 30, 40));
    assertEquals(10, color.Red);
    assertEquals(20, color.Green);
    assertEquals(30, color.Blue);
    assertEquals(40, color.White);
    assertEquals(
        AnimationDirectionValue.Forward, CANdleConversions.toDirection(AnimationDirection.FORWARD));
    assertEquals(
        AnimationDirectionValue.Backward,
        CANdleConversions.toDirection(AnimationDirection.BACKWARD));
    assertEquals(LarsonBounceValue.Front, CANdleConversions.toBounce(BounceMode.FRONT));
    assertEquals(LarsonBounceValue.Center, CANdleConversions.toBounce(BounceMode.CENTER));
    assertEquals(LarsonBounceValue.Back, CANdleConversions.toBounce(BounceMode.BACK));
  }
}
