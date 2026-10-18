package edu.wpi.team190.gompeilib.io.led.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.led.config.LEDConfig;
import edu.wpi.team190.gompeilib.io.led.inputs.LEDTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDColor;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRange;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRequest;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

public class LEDIOCANdleTest extends GompeiLibTestBase {
  private final CANdleConfigurator configurator = mock(CANdleConfigurator.class);

  private static LEDConfig config() {
    return LEDConfig.builder().withCanId(4).withBus(CANBusId.named("rio")).build();
  }

  private void withCANdle(IOTestSupport.PhoenixBody<CANdle> body) throws Exception {
    when(configurator.apply(any(CANdleConfiguration.class), anyDouble())).thenReturn(StatusCode.OK);
    IOTestSupport.withMockedPhoenix(
        CANdle.class,
        (mock, context) -> when(mock.getConfigurator()).thenReturn(configurator),
        body);
  }

  @Test
  public void testFillsEveryTelemetryField() throws Exception {
    withCANdle(
        mocks -> {
          LEDIOCANdle io = new LEDIOCANdle(config());
          LEDTelemetryAutoLogged telemetry = new LEDTelemetryAutoLogged();

          io.updateInputs(telemetry);

          verify(configurator).apply(any(CANdleConfiguration.class), eq(0.25));
          verify(mocks.device(0)).optimizeBusUtilization();
          assertEquals(Set.of(), IOTestSupport.fieldsAtDefault(telemetry));
          assertTrue(io.isConnected());
          assertEquals((1L << CANdleFault.values().length) - 1, telemetry.faults);
        });
  }

  @Test
  public void testCommandsTheDevice() throws Exception {
    withCANdle(
        mocks -> {
          LEDIOCANdle io = new LEDIOCANdle(config());

          io.setControl(new LEDRequest.Solid(new LEDRange(0, 7), new LEDColor(0, 255, 0)));
          io.clearStickyFaults();

          ArgumentCaptor<ControlRequest> sent = ArgumentCaptor.forClass(ControlRequest.class);
          verify(mocks.device(0)).setControl(sent.capture());
          assertInstanceOf(SolidColor.class, sent.getValue());
          verify(mocks.device(0)).clearStickyFaults();
        });
  }
}
