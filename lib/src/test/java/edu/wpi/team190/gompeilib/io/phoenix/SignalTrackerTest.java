package edu.wpi.team190.gompeilib.io.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;

@SuppressWarnings("unchecked") // Mockito mocks of generic types
public class SignalTrackerTest {

  /** A stand-in telemetry class. */
  public static class Telemetry {
    public double position;
    public double temperature;
    public long faults;
    public long stickyFaults;
  }

  @Test
  public void testGroupsSignalsByRateAndRegistersThem() {
    SignalTracker tracker =
        new SignalTracker(
            new SignalRates(Hertz.of(50.0), Hertz.of(4.0), Map.of()),
            Set.of("position"),
            Telemetry.class);
    StatusSignal<Double> position = mock(StatusSignal.class);
    StatusSignal<Double> temperature = mock(StatusSignal.class);
    ParentDevice device = mock(ParentDevice.class);
    CANBus bus = new CANBus("rio");

    assertSame(position, tracker.track("position", position));
    assertSame(temperature, tracker.track("temperature", temperature));

    try (MockedStatic<BaseStatusSignal> signals = mockStatic(BaseStatusSignal.class);
        MockedStatic<PhoenixUtil> phoenix = mockStatic(PhoenixUtil.class)) {
      tracker.apply(device, bus);

      signals.verify(() -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, position));
      signals.verify(() -> BaseStatusSignal.setUpdateFrequencyForAll(4.0, temperature));
      ArgumentCaptor<BaseStatusSignal[]> registered =
          ArgumentCaptor.forClass(BaseStatusSignal[].class);
      phoenix.verify(() -> PhoenixUtil.registerSignals(eq(bus), registered.capture()));
      assertArrayEquals(new BaseStatusSignal[] {position, temperature}, registered.getValue());
    }
    verify(device).optimizeBusUtilization();
  }

  @Test
  public void testTracksFaults() {
    SignalTracker tracker = new SignalTracker(SignalRates.DEFAULT, Set.of(), Telemetry.class);

    FaultSignals<?> faults =
        tracker.trackFaults(new FaultSignalsTest.Device(), FaultSignalsTestFaults.class);

    assertNotNull(faults);
  }

  /** The faults {@link FaultSignalsTest.Device} reports. */
  private enum FaultSignalsTestFaults implements edu.wpi.team190.gompeilib.io.base.HardwareFault {
    HARDWARE,
    FIVE_V_TOO_HIGH;

    @Override
    public int bit() {
      return ordinal();
    }
  }

  @Test
  public void shouldRejectOverridesForUnknownFields() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            new SignalTracker(
                new SignalRates(Hertz.of(50.0), Hertz.of(4.0), Map.of("speed", Hertz.of(1.0))),
                Set.of(),
                Telemetry.class));
  }
}
