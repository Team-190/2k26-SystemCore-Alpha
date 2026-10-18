package edu.wpi.team190.gompeilib.io;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.DifferentialControlModeValue;
import edu.wpi.team190.gompeilib.io.base.HardwareFault;
import edu.wpi.team190.gompeilib.io.led.outputs.AnimationDirection;
import edu.wpi.team190.gompeilib.io.led.outputs.BounceMode;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDColor;
import edu.wpi.team190.gompeilib.io.led.outputs.LEDRange;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.outputs.AxisRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixUtil;
import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.RecordComponent;
import java.lang.reflect.Type;
import java.util.*;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;
import org.mockito.stubbing.Answer;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.*;

/**
 * Shared reflection helpers for the {@code core/io} contract tests: sample values for request
 * records, sealed-hierarchy traversal, telemetry default checks, and Phoenix6 API enumeration.
 */
public final class IOTestSupport {
  private IOTestSupport() {}

  /** The mocks a mocked-Phoenix6-device test body can inspect. */
  public record MockedPhoenix<D>(
      MockedConstruction<D> devices,
      MockedStatic<BaseStatusSignal> signals,
      MockedStatic<PhoenixUtil> phoenix) {
    /** Returns the {@code index}th device constructed. */
    public D device(int index) {
      return devices.constructed().get(index);
    }
  }

  /** A test body that may throw. */
  public interface PhoenixBody<D> {
    void run(MockedPhoenix<D> mocks) throws Exception;
  }

  /**
   * Runs {@code body} with every {@code type} construction mocked: each signal getter returns a
   * {@link #phoenixSignalAnswer() sample signal}, every signal reports good, and signal
   * registration is a no-op. {@code initializer} stubs anything else, such as the configurator.
   */
  public static <D> void withMockedPhoenix(
      Class<D> type, MockedConstruction.MockInitializer<D> initializer, PhoenixBody<D> body)
      throws Exception {
    try (MockedConstruction<D> devices =
            mockConstruction(
                type, withSettings().defaultAnswer(phoenixSignalAnswer()), initializer);
        MockedStatic<BaseStatusSignal> signals = mockStatic(BaseStatusSignal.class);
        MockedStatic<PhoenixUtil> phoenix = mockStatic(PhoenixUtil.class, CALLS_REAL_METHODS)) {
      phoenix
          .when(() -> PhoenixUtil.registerSignals(any(), any(BaseStatusSignal[].class)))
          .thenAnswer(invocation -> null);
      signals
          .when(() -> BaseStatusSignal.isAllGood(any(BaseStatusSignal[].class)))
          .thenReturn(true);
      body.run(new MockedPhoenix<>(devices, signals, phoenix));
    }
  }

  /** The documented defaults motor request short constructors fill in, by component name. */
  public static final Map<String, Object> MOTOR_REQUEST_DEFAULTS =
      Map.of(
          "foc", true,
          "maxAbsDutyCycle", 1.0,
          "slot", GainSlot.ZERO,
          "options", RequestOptions.DEFAULT,
          "updateFrequency", RequestOptions.DEFAULT_UPDATE_FREQUENCY);

  /** Returns every concrete record under a sealed interface, keyed by simple name. */
  public static Map<String, Class<?>> leafRecordsByName(Class<?> sealed) {
    Map<String, Class<?>> byName = new TreeMap<>();
    for (Class<?> leaf : leafRecords(sealed)) {
      assertNull(byName.put(leaf.getSimpleName(), leaf), "duplicate " + leaf.getSimpleName());
    }
    return byName;
  }

  /**
   * Returns every concrete record under a sealed interface, recursing through nested sealed
   * interfaces.
   */
  public static List<Class<?>> leafRecords(Class<?> sealed) {
    List<Class<?>> leaves = new ArrayList<>();
    collect(sealed, leaves);
    leaves.sort(Comparator.comparing(Class::getName));
    return leaves;
  }

  private static void collect(Class<?> type, List<Class<?>> out) {
    if (type.isRecord()) {
      out.add(type);
      return;
    }
    assertTrue(type.isSealed(), type.getName() + " must be sealed or a record");
    for (Class<?> sub : type.getPermittedSubclasses()) {
      collect(sub, out);
    }
  }

  /** Returns a sample value of the given type, distinct for each {@code seed}. */
  public static Object sample(Class<?> type, int seed) {
    if (type == double.class) return seed + 0.25;
    if (type == int.class) return seed;
    if (type == boolean.class) return true;
    if (type == Angle.class) return Radians.of(seed);
    if (type == AngularVelocity.class) return RadiansPerSecond.of(seed);
    if (type == AngularAcceleration.class) return RadiansPerSecondPerSecond.of(seed);
    if (type == Velocity.class) return RadiansPerSecondPerSecond.per(Second).of(seed);
    if (type == Per.class) return VoltsPerRadianPerSecond.ofNative(seed);
    if (type == Voltage.class) return Volts.of(seed);
    if (type == Current.class) return Amps.of(seed);
    if (type == Frequency.class) return Hertz.of(seed);
    if (type == GainSlot.class) return GainSlot.values()[seed % GainSlot.values().length];
    if (type == MotorAlignment.class) return MotorAlignment.OPPOSED;
    if (type == RequestOptions.class)
      return new RequestOptions(true, true, true, true, true, true, Hertz.of(seed));
    if (type == AxisRequest.class) return new OpenLoopRequest.VoltageOutput(Volts.of(seed));
    if (type == LEDRange.class) return new LEDRange(seed, seed + 5);
    if (type == LEDColor.class) return new LEDColor(seed % 256, 1, 2, 3);
    if (type == AnimationDirection.class) return AnimationDirection.BACKWARD;
    if (type == BounceMode.class) return BounceMode.CENTER;
    throw new IllegalArgumentException("No sample for " + type.getName());
  }

  /** Builds arguments for a constructor, one distinct sample per parameter. */
  public static Object[] sampleArgs(Constructor<?> ctor) {
    Class<?>[] types = ctor.getParameterTypes();
    Object[] args = new Object[types.length];
    for (int i = 0; i < types.length; i++) {
      args[i] = sample(types[i], i + 1);
    }
    return args;
  }

  /** Returns the canonical constructor of a record. */
  public static Constructor<?> canonical(Class<?> record) throws ReflectiveOperationException {
    Class<?>[] types =
        Arrays.stream(record.getRecordComponents())
            .map(RecordComponent::getType)
            .toArray(Class<?>[]::new);
    return record.getDeclaredConstructor(types);
  }

  /** Builds a record through its canonical constructor with sample values. */
  public static Object sampleRecord(Class<?> record) throws ReflectiveOperationException {
    return canonical(record).newInstance(sampleArgs(canonical(record)));
  }

  /** Reads a record's component values, in declaration order. */
  public static List<Object> components(Object record) throws ReflectiveOperationException {
    List<Object> values = new ArrayList<>();
    for (RecordComponent c : record.getClass().getRecordComponents()) {
      values.add(c.getAccessor().invoke(record));
    }
    return values;
  }

  /**
   * Asserts that every record's canonical constructor stores its arguments, and that records with
   * equal components are equal.
   */
  public static void assertCanonicalConstructors(Class<?> sealed)
      throws ReflectiveOperationException {
    for (Class<?> record : leafRecords(sealed)) {
      Constructor<?> ctor = canonical(record);
      Object[] args = sampleArgs(ctor);
      Object a = ctor.newInstance(args);
      Object b = ctor.newInstance(args);
      assertEquals(Arrays.asList(args), components(a), record.getSimpleName());
      assertEquals(a, b, record.getSimpleName());
      assertEquals(a.hashCode(), b.hashCode(), record.getSimpleName());
    }
  }

  /**
   * Asserts that every non-canonical constructor keeps its arguments and fills every other
   * component with its documented default.
   */
  public static void assertShortConstructors(Class<?> sealed, Map<String, Object> defaults)
      throws ReflectiveOperationException {
    for (Class<?> record : leafRecords(sealed)) {
      Constructor<?> canonical = canonical(record);
      for (Constructor<?> ctor : record.getDeclaredConstructors()) {
        if (ctor.equals(canonical)) {
          continue;
        }
        Object[] args = sampleArgs(ctor);
        Object instance = ctor.newInstance(args);
        RecordComponent[] comps = record.getRecordComponents();
        List<Object> values = components(instance);
        List<Object> remaining = new ArrayList<>(Arrays.asList(args));
        for (int i = 0; i < comps.length; i++) {
          if (remaining.remove(values.get(i))) {
            continue;
          }
          assertDefault(record.getSimpleName(), comps[i].getName(), values.get(i), defaults);
        }
        assertTrue(remaining.isEmpty(), record.getSimpleName() + " dropped arguments " + remaining);
      }
    }
  }

  private static void assertDefault(
      String owner, String name, Object value, Map<String, Object> defaults) {
    String where = owner + "." + name;
    if (defaults.containsKey(name)) {
      assertEquals(defaults.get(name), value, where);
    } else if (value instanceof Measure<?> m) {
      assertEquals(0.0, m.magnitude(), where);
    } else if (value instanceof Double d) {
      assertEquals(0.0, d, where);
    } else if (value instanceof Boolean bool) {
      assertFalse(bool, where);
    } else {
      fail(where + " has no documented default: " + value);
    }
  }

  /** Asserts every public field of a telemetry object is at its zero value. */
  public static void assertZeroDefaults(Object telemetry) throws IllegalAccessException {
    assertEquals(
        publicFieldNames(telemetry.getClass()),
        fieldsAtDefault(telemetry),
        telemetry.getClass().getSimpleName() + " fields at their zero value");
  }

  /**
   * Returns the public fields of a telemetry object still at their zero value: zero measures and
   * numbers, false, {@link GainSlot#ZERO}, {@code UNKNOWN} enums, and empty arrays.
   */
  public static Set<String> fieldsAtDefault(Object telemetry) throws IllegalAccessException {
    Set<String> atDefault = new TreeSet<>();
    for (Field field : publicFields(telemetry.getClass())) {
      Object value = field.get(telemetry);
      String where = telemetry.getClass().getSimpleName() + "." + field.getName();
      assertNotNull(value, where);
      if (isZero(value, where)) {
        atDefault.add(field.getName());
      }
    }
    return atDefault;
  }

  private static boolean isZero(Object value, String where) {
    if (value instanceof Measure<?> m) {
      return m.magnitude() == 0.0;
    } else if (value instanceof Boolean b) {
      return !b;
    } else if (value instanceof Number n) {
      return n.doubleValue() == 0.0;
    } else if (value instanceof GainSlot slot) {
      return slot == GainSlot.ZERO;
    } else if (value instanceof Enum<?> e) {
      return e.name().equals("UNKNOWN");
    } else if (value.getClass().isArray()) {
      return java.lang.reflect.Array.getLength(value) == 0;
    }
    fail(where + " has an unexpected type " + value.getClass());
    return false;
  }

  /**
   * A default answer for a mocked Phoenix6 device: every getter returning a {@code StatusSignal<T>}
   * returns a mocked signal whose value is a non-zero sample of {@code T}, chosen from the getter's
   * generic return type. Other methods return Mockito's defaults.
   */
  public static Answer<Object> phoenixSignalAnswer() {
    return invocation -> {
      Method method = invocation.getMethod();
      if (!StatusSignal.class.isAssignableFrom(method.getReturnType())) {
        return RETURNS_DEFAULTS.answer(invocation);
      }
      Type type = ((ParameterizedType) method.getGenericReturnType()).getActualTypeArguments()[0];
      Class<?> raw =
          type instanceof ParameterizedType parameterized
              ? (Class<?>) parameterized.getRawType()
              : (Class<?>) type;
      StatusSignal<?> signal = mock(StatusSignal.class);
      doReturn(phoenixSample(raw, method.getName())).when(signal).getValue();
      return signal;
    };
  }

  /** The value {@link #phoenixSignalAnswer()} reports for a signal of the given type. */
  public static Object phoenixSample(Class<?> type, String getter) {
    if (type == Angle.class) return Rotations.of(1.5);
    if (type == AngularVelocity.class) return RotationsPerSecond.of(2.5);
    if (type == AngularAcceleration.class) return RotationsPerSecondPerSecond.of(3.5);
    if (type == Voltage.class) return Volts.of(6.0);
    if (type == Current.class) return Amps.of(10.0);
    if (type == Temperature.class) return Celsius.of(40.0);
    if (type == Distance.class) return Meters.of(0.5);
    if (type == Time.class) return Seconds.of(0.25);
    if (type == LinearAcceleration.class) return MetersPerSecondPerSecond.of(9.8);
    if (type == Double.class) return 0.25;
    if (type == Integer.class) return 1;
    if (type == Boolean.class) return true;
    if (type == Per.class) {
      return getter.contains("KT")
          ? NewtonMeters.per(Amp).ofNative(0.02)
          : RadiansPerSecond.per(Volt).ofNative(50.0);
    }
    if (type == ControlModeValue.class) return ControlModeValue.PositionVoltageFOC;
    if (type == DifferentialControlModeValue.class) {
      return DifferentialControlModeValue.VelocityTorqueCurrentFOC;
    }
    if (type.isEnum()) {
      // The first known value: asserted limits, enabled robot and device, a real motor type.
      for (Object constant : type.getEnumConstants()) {
        if (!((Enum<?>) constant).name().equalsIgnoreCase("Unknown")) {
          return constant;
        }
      }
    }
    throw new IllegalArgumentException("No Phoenix6 sample for " + type.getName());
  }

  /** Returns the names of a class's public instance fields. */
  public static Set<String> publicFieldNames(Class<?> type) {
    Set<String> names = new TreeSet<>();
    for (Field f : publicFields(type)) {
      names.add(f.getName());
    }
    return names;
  }

  private static List<Field> publicFields(Class<?> type) {
    List<Field> fields = new ArrayList<>();
    for (Field f : type.getFields()) {
      if (!Modifier.isStatic(f.getModifiers())) {
        fields.add(f);
      }
    }
    return fields;
  }

  /**
   * Returns the status signals a Phoenix6 device class publishes, as getter names without the
   * {@code get} prefix, excluding the individual fault getters.
   */
  public static Set<String> statusSignals(Class<?> device) {
    Set<String> names = new TreeSet<>();
    for (Method m : device.getMethods()) {
      if (m.getParameterCount() == 0
          && StatusSignal.class.isAssignableFrom(m.getReturnType())
          && m.getName().startsWith("get")
          && !m.getName().startsWith("getFault_")
          && !m.getName().startsWith("getStickyFault_")) {
        names.add(m.getName().substring(3));
      }
    }
    return names;
  }

  /** Status signals every Phoenix6 device publishes that telemetry intentionally doesn't map. */
  public static final Map<String, String> COMMON_UNMAPPED_SIGNALS =
      Map.of(
          "VersionMajor", "a component of the packed firmwareVersion",
          "VersionMinor", "a component of the packed firmwareVersion",
          "VersionBugfix", "a component of the packed firmwareVersion",
          "VersionBuild", "a component of the packed firmwareVersion",
          "FaultField", "faults are packed from the individual fault getters",
          "StickyFaultField", "sticky faults are packed from the individual fault getters");

  /** Telemetry fields every device fills without a single matching status signal. */
  public static final Map<String, String> COMMON_FIELDS_WITHOUT_SIGNAL =
      Map.of(
          "connected", "derived from the status of the device's signals",
          "faults", "packed from the individual fault getters",
          "stickyFaults", "packed from the individual sticky fault getters");

  /** Returns {@code base} plus {@code extra}, as an unmodifiable map. */
  public static Map<String, String> with(Map<String, String> base, Map<String, String> extra) {
    Map<String, String> merged = new TreeMap<>(base);
    merged.putAll(extra);
    return Collections.unmodifiableMap(merged);
  }

  /**
   * Asserts a telemetry class covers a Phoenix6 device exactly: every status signal is mapped to at
   * least one field or intentionally unmapped, every field is filled from a signal or explicitly
   * has none, and the mapping names only real signals and fields.
   */
  public static void assertTelemetryCoverage(
      Class<?> device,
      Class<?> telemetry,
      Map<String, List<String>> signalToFields,
      Map<String, String> unmappedSignals,
      Map<String, String> fieldsWithoutSignal) {
    Set<String> expectedSignals = new TreeSet<>(signalToFields.keySet());
    expectedSignals.addAll(unmappedSignals.keySet());
    assertEquals(statusSignals(device), expectedSignals, device.getSimpleName() + " signals");

    Set<String> mappedFields = new TreeSet<>();
    for (List<String> fields : signalToFields.values()) {
      for (String field : fields) {
        assertTrue(mappedFields.add(field), field + " is mapped from more than one signal");
      }
    }
    for (String field : fieldsWithoutSignal.keySet()) {
      assertFalse(mappedFields.contains(field), field + " is both mapped and signal-less");
    }
    Set<String> expectedFields = new TreeSet<>(mappedFields);
    expectedFields.addAll(fieldsWithoutSignal.keySet());
    assertEquals(
        publicFieldNames(telemetry), expectedFields, telemetry.getSimpleName() + " fields");
  }

  /** Returns the fault names a Phoenix6 device class publishes, normalized for comparison. */
  public static Set<String> phoenixFaults(Class<?> device) {
    Set<String> names = new TreeSet<>();
    for (Method m : device.getMethods()) {
      if (m.getParameterCount() == 0 && m.getName().startsWith("getFault_")) {
        names.add(normalize(m.getName().substring("getFault_".length())));
      }
    }
    return names;
  }

  /**
   * Asserts a request record and the Phoenix6 request it maps to carry the same settings, in both
   * directions: every public field of {@code phoenix} is a component of {@code record} (through
   * {@code names}), or one of {@code optionFields} carried by the record's {@code options}
   * component; and every component of {@code record} other than {@code options} comes from a field
   * of {@code phoenix}.
   *
   * @param phoenix the Phoenix6 request class
   * @param record the GompeiLib request record
   * @param names Phoenix6 field name to record component name
   * @param optionFields Phoenix6 fields carried by a {@code RequestOptions} component
   */
  public static void assertFieldsCovered(
      Class<?> phoenix, Class<?> record, Map<String, String> names, Set<String> optionFields) {
    Set<String> components = new TreeSet<>();
    for (RecordComponent component : record.getRecordComponents()) {
      components.add(component.getName());
    }
    String where = phoenix.getSimpleName() + " -> " + record.getSimpleName();
    Set<String> covered = new TreeSet<>();
    for (Field field : phoenix.getFields()) {
      if (Modifier.isStatic(field.getModifiers())) {
        continue;
      }
      String name = field.getName();
      String component = names.get(name);
      if (component != null && components.contains(component)) {
        covered.add(component);
      } else if (optionFields.contains(name) && components.contains("options")) {
        covered.add("options");
      } else {
        fail(where + ": Phoenix6 field " + name + " has no matching record component");
      }
    }
    components.remove("options");
    covered.remove("options");
    assertEquals(components, covered, where + ": record components with no Phoenix6 field");
  }

  /** Returns the control request types a Phoenix6 device class accepts in {@code setControl}. */
  public static Set<Class<?>> controlRequests(Class<?> device, Set<Class<?>> excluded) {
    Set<Class<?>> types = new TreeSet<>(Comparator.comparing(Class::getName));
    for (Method m : device.getMethods()) {
      if (m.getName().equals("setControl") && m.getParameterCount() == 1) {
        Class<?> type = m.getParameterTypes()[0];
        if (!excluded.contains(type)) {
          types.add(type);
        }
      }
    }
    return types;
  }

  /**
   * Asserts a vendor fault enum names exactly the faults its Phoenix6 device publishes, with unique
   * bits in {@code [0, 64)}.
   */
  public static <F extends Enum<F> & HardwareFault> void assertFaultEnum(
      Class<F> faults, Class<?> device) {
    Set<String> ours = new TreeSet<>();
    Set<Integer> bits = new HashSet<>();
    for (F fault : faults.getEnumConstants()) {
      ours.add(normalize(fault.name()));
      assertTrue(fault.bit() >= 0 && fault.bit() < 64, fault.name());
      assertTrue(bits.add(fault.bit()), "duplicate bit " + fault.bit());
      assertEquals(1L << fault.bit(), fault.mask());
      assertEquals(fault, Enum.valueOf(faults, fault.name()));
    }
    assertEquals(phoenixFaults(device), ours);
  }

  /** Asserts every constant of an enum round-trips through {@code valueOf}. */
  public static <E extends Enum<E>> void assertEnumRoundTrip(Class<E> type, String... expected) {
    E[] values = type.getEnumConstants();
    assertEquals(
        List.of(expected), Arrays.stream(values).map(Enum::name).toList(), type.getSimpleName());
    for (E value : values) {
      assertEquals(value, Enum.valueOf(type, value.name()));
    }
  }

  private static String normalize(String name) {
    // Enum constants cannot start with a digit, so "5V" is spelled out as "FIVE_V".
    return name.replace("_", "").toUpperCase(Locale.ROOT).replace("5V", "FIVEV");
  }
}
