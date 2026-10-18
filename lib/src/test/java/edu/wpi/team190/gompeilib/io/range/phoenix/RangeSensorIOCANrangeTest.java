package edu.wpi.team190.gompeilib.io.range.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.range.config.RangeSensorConfig;
import edu.wpi.team190.gompeilib.io.range.inputs.RangeSensorTelemetryAutoLogged;
import java.util.Set;
import org.junit.jupiter.api.Test;

public class RangeSensorIOCANrangeTest extends GompeiLibTestBase {
  private final CANrangeConfigurator configurator = mock(CANrangeConfigurator.class);

  private static RangeSensorConfig config() {
    return RangeSensorConfig.builder().withCanId(5).withBus(CANBusId.named("rio")).build();
  }

  private void withCANrange(IOTestSupport.PhoenixBody<CANrange> body) throws Exception {
    when(configurator.apply(any(CANrangeConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);
    IOTestSupport.withMockedPhoenix(
        CANrange.class,
        (mock, context) -> when(mock.getConfigurator()).thenReturn(configurator),
        body);
  }

  @Test
  public void testFillsEveryTelemetryField() throws Exception {
    withCANrange(
        mocks -> {
          RangeSensorIOCANrange io = new RangeSensorIOCANrange(config());
          RangeSensorTelemetryAutoLogged telemetry = new RangeSensorTelemetryAutoLogged();

          io.updateInputs(telemetry);

          verify(configurator).apply(any(CANrangeConfiguration.class), eq(0.25));
          verify(mocks.device(0)).optimizeBusUtilization();
          assertEquals(Set.of(), IOTestSupport.fieldsAtDefault(telemetry));
          assertTrue(io.isConnected());
          assertEquals((1L << CANrangeFault.values().length) - 1, telemetry.faults);
        });
  }

  @Test
  public void testClearsStickyFaults() throws Exception {
    withCANrange(
        mocks -> {
          RangeSensorIOCANrange io = new RangeSensorIOCANrange(config());

          io.clearStickyFaults();

          verify(mocks.device(0)).clearStickyFaults();
        });
  }
}
