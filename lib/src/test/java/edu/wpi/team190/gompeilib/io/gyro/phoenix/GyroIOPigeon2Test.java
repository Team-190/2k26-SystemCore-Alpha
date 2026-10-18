package edu.wpi.team190.gompeilib.io.gyro.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.gyro.config.GyroConfig;
import edu.wpi.team190.gompeilib.io.gyro.inputs.GyroTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixOdometryThread;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;
import org.junit.jupiter.api.Test;

@SuppressWarnings("unchecked") // Mockito mocks of generic types
public class GyroIOPigeon2Test extends GompeiLibTestBase {
  private final Pigeon2Configurator configurator = mock(Pigeon2Configurator.class);

  private static GyroConfig config() {
    return GyroConfig.builder().withCanId(9).withBus(CANBusId.named("Drive")).build();
  }

  private void withPigeon(IOTestSupport.PhoenixBody<Pigeon2> body) throws Exception {
    when(configurator.apply(any(Pigeon2Configuration.class), anyDouble()))
        .thenReturn(StatusCode.OK);
    IOTestSupport.withMockedPhoenix(
        Pigeon2.class,
        (mock, context) -> when(mock.getConfigurator()).thenReturn(configurator),
        body);
  }

  @Test
  public void testConfiguresAndZeroesTheDevice() throws Exception {
    withPigeon(
        mocks -> {
          new GyroIOPigeon2(config());

          verify(configurator).apply(any(Pigeon2Configuration.class), eq(0.25));
          verify(configurator).setYaw(Degrees.of(0.0));
          verify(mocks.device(0)).optimizeBusUtilization();
        });
  }

  @Test
  public void testFillsEveryTelemetryField() throws Exception {
    withPigeon(
        mocks -> {
          GyroIOPigeon2 io = new GyroIOPigeon2(config());
          GyroTelemetryAutoLogged telemetry = new GyroTelemetryAutoLogged();

          io.updateInputs(telemetry);

          // Pigeon2 publishes no calibration state, and without an odometry thread there are no
          // odometry samples.
          assertEquals(
              Set.of("isCalibrating", "odometryTimestamps", "odometryYawPositions"),
              IOTestSupport.fieldsAtDefault(telemetry));
          assertTrue(io.isConnected());
          assertEquals((1L << Pigeon2Fault.values().length) - 1, telemetry.faults);
        });
  }

  @Test
  public void testDrainsOdometrySamplesAndReportsTheHeadingTimestamp() throws Exception {
    PhoenixOdometryThread thread = mock(PhoenixOdometryThread.class);
    Queue<Double> timestamps = new ArrayBlockingQueue<>(20);
    Queue<Double> yaws = new ArrayBlockingQueue<>(20);
    when(thread.makeTimestampQueue()).thenReturn(timestamps);
    when(thread.registerSignal(any(StatusSignal.class))).thenReturn(yaws);
    List<Long> headingTimestamps = new ArrayList<>();
    withPigeon(
        mocks -> {
          GyroIOPigeon2 io = new GyroIOPigeon2(config(), thread, headingTimestamps::add);
          GyroTelemetryAutoLogged telemetry = new GyroTelemetryAutoLogged();
          timestamps.addAll(List.of(1.0, 2.0));
          yaws.addAll(List.of(10.0, 20.0));

          io.updateInputs(telemetry);

          assertArrayEquals(new double[] {1.0, 2.0}, telemetry.odometryTimestamps);
          assertEquals(2, telemetry.odometryYawPositions.length);
          assertEquals(20.0, telemetry.odometryYawPositions[1].in(Degrees), 1e-9);
          assertTrue(timestamps.isEmpty());
          assertTrue(yaws.isEmpty());
          assertEquals(1, headingTimestamps.size());
        });
  }

  @Test
  public void testCommandsTheDevice() throws Exception {
    withPigeon(
        mocks -> {
          GyroIOPigeon2 io = new GyroIOPigeon2(config());

          io.setYaw(Degrees.of(45.0));
          io.clearStickyFaults();

          verify(mocks.device(0)).setYaw(Degrees.of(45.0));
          verify(mocks.device(0)).clearStickyFaults();
        });
  }
}
