package edu.wpi.team190.gompeilib.core.utility.tunable;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.team190.gompeilib.core.GompeiLibTestBase;
import java.util.concurrent.atomic.AtomicBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.tunable.MockTunableBackend;
import org.wpilib.tunable.Tunable;
import org.wpilib.tunable.TunableConfig;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.TunableRegistry;
import org.wpilib.tunable.TunableTable;

public class GompeiTunablesTest extends GompeiLibTestBase {
  private MockTunableBackend backend;

  @BeforeEach
  public void setUp() {
    TunableRegistry.reset();
    backend = new MockTunableBackend();
    TunableRegistry.registerBackend("", backend);
  }

  @AfterEach
  public void tearDown() {
    TunableRegistry.reset();
  }

  @Test
  public void testConfig() {
    TunableConfig cfgWithoutCallback = GompeiTunables.config(null);
    assertNotNull(cfgWithoutCallback);

    AtomicBoolean called = new AtomicBoolean(false);
    TunableConfig cfgWithCallback = GompeiTunables.config(() -> called.set(true));
    assertNotNull(cfgWithCallback);
  }

  @Test
  public void testTable() {
    assertNull(GompeiTunables.table(null));
    TunableTable table = GompeiTunables.table("TestGroup");
    assertNotNull(table);
  }

  @Test
  public void testValueWithTable() {
    TunableTable table = GompeiTunables.table("TestTable");
    Tunable<String> tunable = GompeiTunables.value(table, "name", "hello", null);
    assertNotNull(tunable);
    assertEquals("hello", tunable.get());

    Tunable<String> unpublished = GompeiTunables.value(null, "unpublished", "world", null);
    assertNotNull(unpublished);
    assertEquals("world", unpublished.get());
  }

  @Test
  public void testValueStandalone() {
    Tunable<String> tunable = GompeiTunables.value("standaloneKey", "testVal");
    assertNotNull(tunable);
    assertEquals("testVal", tunable.get());
  }

  @Test
  public void testNumber() {
    TunableTable table = GompeiTunables.table("TestNumbers");
    TunableDouble num = GompeiTunables.number(table, "speed", 42.0, null);
    assertNotNull(num);
    assertEquals(42.0, num.get(), 1e-6);

    TunableDouble unpubNum = GompeiTunables.number(null, "speed2", 10.0, null);
    assertNotNull(unpubNum);
    assertEquals(10.0, unpubNum.get(), 1e-6);
  }
}
