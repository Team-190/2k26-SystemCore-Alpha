package edu.wpi.team190.gompeilib.io.base;

import static org.junit.jupiter.api.Assertions.*;

import java.util.EnumSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

public class HardwareFaultTest {

  private enum TestFault implements HardwareFault {
    LOW(0),
    MIDDLE(17),
    HIGH(63);

    private final int bit;

    TestFault(int bit) {
      this.bit = bit;
    }

    @Override
    public int bit() {
      return bit;
    }
  }

  @Test
  public void testMask() {
    assertEquals(1L, TestFault.LOW.mask());
    assertEquals(1L << 17, TestFault.MIDDLE.mask());
    assertEquals(Long.MIN_VALUE, TestFault.HIGH.mask());
  }

  @Test
  public void testHasFault() {
    long faults = TestFault.LOW.mask() | TestFault.HIGH.mask();

    assertTrue(HardwareFault.hasFault(faults, TestFault.LOW));
    assertTrue(HardwareFault.hasFault(faults, TestFault.HIGH));
    assertFalse(HardwareFault.hasFault(faults, TestFault.MIDDLE));
    assertFalse(HardwareFault.hasFault(0L, TestFault.LOW));
    assertFalse(HardwareFault.hasFault(faults, null));
  }

  @Test
  public void testActiveFaults() {
    assertEquals(Set.of(), HardwareFault.activeFaults(0L, TestFault.class));
    assertEquals(
        EnumSet.of(TestFault.MIDDLE), HardwareFault.activeFaults(1L << 17, TestFault.class));
    assertEquals(EnumSet.allOf(TestFault.class), HardwareFault.activeFaults(-1L, TestFault.class));
  }
}
