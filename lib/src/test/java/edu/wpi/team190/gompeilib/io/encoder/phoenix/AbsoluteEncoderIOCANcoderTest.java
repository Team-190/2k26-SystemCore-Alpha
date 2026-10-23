package edu.wpi.team190.gompeilib.io.encoder.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.team190.gompeilib.GompeiLib;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixUtil;
import java.util.Set;
import org.junit.jupiter.api.Test;

public class AbsoluteEncoderIOCANcoderTest extends GompeiLibTestBase {
  private final CANcoderConfigurator configurator = mock(CANcoderConfigurator.class);

  private static AbsoluteEncoderConfig config() {
    return AbsoluteEncoderConfig.builder().withCanId(3).withBus(CANBusId.named("rio")).build();
  }

  private void withCANcoder(IOTestSupport.PhoenixBody<CANcoder> body) throws Exception {
    when(configurator.apply(any(CANcoderConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);
    IOTestSupport.withMockedPhoenix(
        CANcoder.class,
        (mock, context) -> when(mock.getConfigurator()).thenReturn(configurator),
        body);
  }

  @Test
  public void testConfiguresAndRegistersTheDevice() throws Exception {
    withCANcoder(
        mocks -> {
          new AbsoluteEncoderIOCANcoder(config());

          verify(configurator).apply(any(CANcoderConfiguration.class), eq(0.25));
          verify(mocks.device(0)).optimizeBusUtilization();
          mocks
              .signals()
              .verify(
                  () ->
                      BaseStatusSignal.setUpdateFrequencyForAll(
                          eq(1.0 / GompeiLib.getLoopPeriod()), any(BaseStatusSignal[].class)));
          mocks
              .phoenix()
              .verify(
                  () ->
                      PhoenixUtil.registerSignals(
                          eq(new CANBus("rio")), any(BaseStatusSignal[].class)));
        });
  }

  @Test
  public void testFillsEveryTelemetryField() throws Exception {
    withCANcoder(
        mocks -> {
          AbsoluteEncoderIOCANcoder io = new AbsoluteEncoderIOCANcoder(config());
          AbsoluteEncoderTelemetryAutoLogged telemetry = new AbsoluteEncoderTelemetryAutoLogged();

          io.updateInputs(telemetry);

          assertEquals(Set.of(), IOTestSupport.fieldsAtDefault(telemetry));
          assertTrue(io.isConnected());
          assertEquals((1L << CANcoderFault.values().length) - 1, telemetry.faults);
        });
  }

  @Test
  public void testCommandsTheDevice() throws Exception {
    withCANcoder(
        mocks -> {
          AbsoluteEncoderIOCANcoder io = new AbsoluteEncoderIOCANcoder(config());

          io.setPosition(Rotations.of(0.5));
          io.clearStickyFaults();

          verify(mocks.device(0)).setPosition(Rotations.of(0.5));
          verify(mocks.device(0)).clearStickyFaults();
        });
  }
}
