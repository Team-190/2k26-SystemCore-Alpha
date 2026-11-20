package edu.wpi.team190.gompeilib.io.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.Hertz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.team190.gompeilib.io.base.HardwareFault;
import edu.wpi.team190.gompeilib.io.base.Signal;
import edu.wpi.team190.gompeilib.io.base.SignalGroup;
import edu.wpi.team190.gompeilib.io.base.SignalRates;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;
import org.wpilib.units.measure.Frequency;

@SuppressWarnings("unchecked") // Mockito mocks of generic types
public class SignalTrackerTest {

  /** Stand-in groups. */
  public enum Groups implements SignalGroup {
    MOTION(50.0),
    DIAGNOSTIC(4.0);

    private final double hertz;

    Groups(double hertz) {
      this.hertz = hertz;
    }

    @Override
    public Frequency defaultRate() {
      return Hertz.of(hertz);
    }
  }

  /** Stand-in signals. */
  public enum Signals implements Signal<Groups> {
    POSITION(Groups.MOTION),
    TEMPERATURE(Groups.DIAGNOSTIC),
    FAULTS(Groups.DIAGNOSTIC),
    STICKY_FAULTS(Groups.DIAGNOSTIC);

    private final Groups group;

    Signals(Groups group) {
      this.group = group;
    }

    @Override
    public Groups group() {
      return group;
    }
  }

  private static SignalRates<Signals, Groups> defaults() {
    return SignalRates.defaults(Signals.class, Groups.class);
  }

  @Test
  public void testGroupsSignalsByRateAndRegistersThem() {
    SignalTracker<Signals, Groups> tracker = new SignalTracker<>(defaults());
    StatusSignal<Double> position = mock(StatusSignal.class);
    StatusSignal<Double> temperature = mock(StatusSignal.class);
    ParentDevice device = mock(ParentDevice.class);
    CANBus bus = new CANBus("rio");

    assertSame(position, tracker.track(Signals.POSITION, position));
    assertSame(temperature, tracker.track(Signals.TEMPERATURE, temperature));

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
  public void testSignalOverridesApply() {
    SignalTracker<Signals, Groups> tracker =
        new SignalTracker<>(defaults().withSignal(Signals.POSITION, Hertz.of(250.0)));
    StatusSignal<Double> position = mock(StatusSignal.class);
    tracker.track(Signals.POSITION, position);

    try (MockedStatic<BaseStatusSignal> signals = mockStatic(BaseStatusSignal.class);
        MockedStatic<PhoenixUtil> phoenix = mockStatic(PhoenixUtil.class)) {
      tracker.apply(mock(ParentDevice.class), new CANBus("rio"));

      signals.verify(() -> BaseStatusSignal.setUpdateFrequencyForAll(250.0, position));
    }
  }

  @Test
  public void testOffSignalsAreDisabledAndNotRefreshed() {
    SignalTracker<Signals, Groups> tracker =
        new SignalTracker<>(defaults().withGroup(Groups.DIAGNOSTIC, SignalRates.OFF));
    StatusSignal<Double> position = mock(StatusSignal.class);
    StatusSignal<Double> temperature = mock(StatusSignal.class);
    ParentDevice device = mock(ParentDevice.class);
    CANBus bus = new CANBus("rio");

    tracker.track(Signals.POSITION, position);
    tracker.track(Signals.TEMPERATURE, temperature);

    try (MockedStatic<BaseStatusSignal> signals = mockStatic(BaseStatusSignal.class);
        MockedStatic<PhoenixUtil> phoenix = mockStatic(PhoenixUtil.class)) {
      tracker.apply(device, bus);

      signals.verify(() -> BaseStatusSignal.setUpdateFrequencyForAll(0.0, temperature));
      ArgumentCaptor<BaseStatusSignal[]> registered =
          ArgumentCaptor.forClass(BaseStatusSignal[].class);
      phoenix.verify(() -> PhoenixUtil.registerSignals(eq(bus), registered.capture()));
      assertArrayEquals(new BaseStatusSignal[] {position}, registered.getValue());
    }
  }

  @Test
  public void testTracksFaultsAtTheirSignalsRates() {
    SignalTracker<Signals, Groups> tracker =
        new SignalTracker<>(defaults().withSignal(Signals.STICKY_FAULTS, SignalRates.OFF));

    FaultSignals<?> faults =
        tracker.trackFaults(
            new FaultSignalsTest.Device(),
            FaultSignalsTestFaults.class,
            Signals.FAULTS,
            Signals.STICKY_FAULTS);

    assertNotNull(faults);
    try (MockedStatic<BaseStatusSignal> signals = mockStatic(BaseStatusSignal.class);
        MockedStatic<PhoenixUtil> phoenix = mockStatic(PhoenixUtil.class)) {
      tracker.apply(mock(ParentDevice.class), new CANBus("rio"));

      ArgumentCaptor<BaseStatusSignal[]> registered =
          ArgumentCaptor.forClass(BaseStatusSignal[].class);
      phoenix.verify(() -> PhoenixUtil.registerSignals(any(), registered.capture()));
      // Only the two live fault signals are refreshed; the sticky ones are off.
      assertEquals(2, registered.getValue().length);
    }
  }

  /** The faults {@link FaultSignalsTest.Device} reports. */
  private enum FaultSignalsTestFaults implements HardwareFault {
    HARDWARE,
    FIVE_V_TOO_HIGH;

    @Override
    public int bit() {
      return ordinal();
    }
  }
}
