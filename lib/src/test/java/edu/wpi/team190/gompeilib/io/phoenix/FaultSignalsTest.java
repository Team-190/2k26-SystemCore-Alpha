package edu.wpi.team190.gompeilib.io.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.team190.gompeilib.io.base.HardwareFault;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

@SuppressWarnings("unchecked") // Mockito mocks of generic types
public class FaultSignalsTest {

  private enum TestFault implements HardwareFault {
    HARDWARE(0),
    FIVE_V_TOO_HIGH(5);

    private final int bit;

    TestFault(int bit) {
      this.bit = bit;
    }

    @Override
    public int bit() {
      return bit;
    }
  }

  private enum MissingFault implements HardwareFault {
    HARDWARE(0),
    FIVE_V_TOO_HIGH(1),
    NOT_ON_DEVICE(2);

    private final int bit;

    MissingFault(int bit) {
      this.bit = bit;
    }

    @Override
    public int bit() {
      return bit;
    }
  }

  private static StatusSignal<Boolean> signal(boolean value) {
    StatusSignal<Boolean> signal = mock(StatusSignal.class);
    when(signal.getValue()).thenReturn(value);
    return signal;
  }

  /** A stand-in device whose hardware fault and sticky 5 V fault are active. */
  public static class Device {
    public StatusSignal<Boolean> getFault_Hardware() {
      return signal(true);
    }

    public StatusSignal<Boolean> getFault_5VTooHigh() {
      return signal(false);
    }

    public StatusSignal<Boolean> getStickyFault_Hardware() {
      return signal(false);
    }

    public StatusSignal<Boolean> getStickyFault_5VTooHigh() {
      return signal(true);
    }

    /** Not a fault getter: it takes an argument. */
    public StatusSignal<Boolean> getFault_Hardware(double timeout) {
      return signal(true);
    }

    public StatusSignal<Boolean> getVersion() {
      return signal(true);
    }
  }

  /** A device with a fault the enum doesn't name. */
  public static class ExtraFaultDevice extends Device {
    public StatusSignal<Boolean> getFault_Unexpected() {
      return signal(true);
    }
  }

  /** A device that reports live faults but no sticky faults. */
  public static class LiveOnlyDevice {
    public StatusSignal<Boolean> getFault_Hardware() {
      return signal(true);
    }

    public StatusSignal<Boolean> getFault_5VTooHigh() {
      return signal(true);
    }
  }

  /** A device whose fault getter fails. */
  public static class BrokenDevice extends Device {
    @Override
    public StatusSignal<Boolean> getFault_Hardware() {
      throw new IllegalStateException("broken");
    }
  }

  @Test
  public void testPacksLiveAndStickyFaults() {
    List<String> tracked = new ArrayList<>();
    FaultSignals<TestFault> faults =
        FaultSignals.of(
            new Device(),
            TestFault.class,
            (field, signal) -> {
              tracked.add(field);
              return signal;
            });

    assertEquals(TestFault.HARDWARE.mask(), faults.faults());
    assertEquals(TestFault.FIVE_V_TOO_HIGH.mask(), faults.stickyFaults());
    assertEquals(2, tracked.stream().filter("faults"::equals).count());
    assertEquals(2, tracked.stream().filter("stickyFaults"::equals).count());
  }

  @Test
  public void shouldRejectAFaultTheEnumDoesNotName() {
    assertThrows(
        IllegalStateException.class,
        () -> FaultSignals.of(new ExtraFaultDevice(), TestFault.class, (field, signal) -> signal));
  }

  @Test
  public void shouldRejectAFaultTheDeviceDoesNotReport() {
    assertThrows(
        IllegalStateException.class,
        () -> FaultSignals.of(new Device(), MissingFault.class, (field, signal) -> signal));
  }

  @Test
  public void shouldRejectAFaultWithoutAStickyGetter() {
    assertThrows(
        IllegalStateException.class,
        () -> FaultSignals.of(new LiveOnlyDevice(), TestFault.class, (field, signal) -> signal));
  }

  @Test
  public void shouldWrapAFailingGetter() {
    assertThrows(
        IllegalStateException.class,
        () -> FaultSignals.of(new BrokenDevice(), TestFault.class, (field, signal) -> signal));
  }
}
