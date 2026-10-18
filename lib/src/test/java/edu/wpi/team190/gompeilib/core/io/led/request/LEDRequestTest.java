package edu.wpi.team190.gompeilib.core.io.led.request;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.hardware.core.CoreCANdle;
import edu.wpi.team190.gompeilib.core.io.IOTestSupport;
import edu.wpi.team190.gompeilib.core.io.led.request.LEDAnimation.*;
import edu.wpi.team190.gompeilib.core.io.led.request.LEDRequest.*;
import java.util.*;
import org.junit.jupiter.api.Test;

/** Checks that {@link LEDRequest} covers every CANdle control request in Phoenix6. */
public class LEDRequestTest {

  /** Phoenix6 CANdle requests, by simple name, and the record each maps to. */
  public static final Map<String, Class<? extends LEDRequest>> CANDLE_REQUESTS =
      Map.ofEntries(
          Map.entry("SolidColor", Solid.class),
          Map.entry("ModulateVBatOut", SupplyModulation.class),
          Map.entry("ColorFlowAnimation", ColorFlow.class),
          Map.entry("FireAnimation", Fire.class),
          Map.entry("LarsonAnimation", Larson.class),
          Map.entry("RainbowAnimation", Rainbow.class),
          Map.entry("RgbFadeAnimation", RgbFade.class),
          Map.entry("SingleFadeAnimation", SingleFade.class),
          Map.entry("StrobeAnimation", Strobe.class),
          Map.entry("TwinkleAnimation", Twinkle.class),
          Map.entry("TwinkleOffAnimation", TwinkleOff.class),
          Map.entry("EmptyAnimation", ClearAnimation.class));

  @Test
  public void testCanonicalConstructorsStoreEveryComponent() throws ReflectiveOperationException {
    IOTestSupport.assertCanonicalConstructors(LEDRequest.class);
  }

  @Test
  public void testEveryCANdleRequestIsMapped() {
    Set<String> phoenix = new TreeSet<>();
    for (Class<?> request :
        IOTestSupport.controlRequests(
            CoreCANdle.class, Set.of(ControlRequest.class, EmptyControl.class))) {
      phoenix.add(request.getSimpleName());
    }
    assertEquals(phoenix, new TreeSet<>(CANDLE_REQUESTS.keySet()));
  }

  @Test
  public void testEveryRecordIsUsed() {
    assertEquals(
        new HashSet<>(IOTestSupport.leafRecords(LEDRequest.class)),
        new HashSet<>(CANDLE_REQUESTS.values()));
  }

  /** Fails to compile if an {@link LEDRequest} record is added without a case here. */
  private static String describe(LEDRequest request) {
    return switch (request) {
      case Solid r -> "Solid";
      case SupplyModulation r -> "SupplyModulation";
      case ColorFlow r -> "ColorFlow";
      case Fire r -> "Fire";
      case Larson r -> "Larson";
      case Rainbow r -> "Rainbow";
      case RgbFade r -> "RgbFade";
      case SingleFade r -> "SingleFade";
      case Strobe r -> "Strobe";
      case Twinkle r -> "Twinkle";
      case TwinkleOff r -> "TwinkleOff";
      case ClearAnimation r -> "ClearAnimation";
    };
  }

  @Test
  public void testExhaustiveSwitchCoversEveryRecord() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(LEDRequest.class)) {
      LEDRequest request = (LEDRequest) IOTestSupport.sampleRecord(record);
      assertEquals(record.getSimpleName(), describe(request));
    }
  }
}
