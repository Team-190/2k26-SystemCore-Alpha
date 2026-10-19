package edu.wpi.team190.gompeilib.io.motor.outputs;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest.*;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest.*;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest.*;
import edu.wpi.team190.gompeilib.io.motor.outputs.PositionRequest.*;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest.*;
import java.util.*;
import org.junit.jupiter.api.Test;

/**
 * Checks that {@link MotorRequest} covers every TalonFX control request in Phoenix6, and that
 * {@link Differential} accepts exactly the axis pairs Phoenix6 offers.
 */
public class MotorRequestTest {

  private static final String COMPOUND_PACKAGE = "com.ctre.phoenix6.controls.compound";

  /** Phoenix6 TalonFX single requests, by simple name, and the record each maps to. */
  public static final Map<String, Class<? extends MotorRequest>> TALONFX_SINGLE_REQUESTS =
      singleRequests();

  /**
   * Phoenix6 standalone differential requests, by simple name, and the (average, difference) pair
   * each one is equivalent to.
   */
  public static final Map<String, List<Class<? extends AxisRequest>>>
      TALONFX_STANDALONE_DIFFERENTIALS =
          Map.ofEntries(
              Map.entry(
                  "DifferentialDutyCycle", List.of(DutyCycleOutput.class, PositionDutyCycle.class)),
              Map.entry("DifferentialVoltage", List.of(VoltageOutput.class, PositionVoltage.class)),
              Map.entry(
                  "DifferentialPositionDutyCycle",
                  List.of(PositionDutyCycle.class, PositionDutyCycle.class)),
              Map.entry(
                  "DifferentialPositionVoltage",
                  List.of(PositionVoltage.class, PositionVoltage.class)),
              Map.entry(
                  "DifferentialVelocityDutyCycle",
                  List.of(VelocityDutyCycle.class, PositionDutyCycle.class)),
              Map.entry(
                  "DifferentialVelocityVoltage",
                  List.of(VelocityVoltage.class, PositionVoltage.class)),
              Map.entry(
                  "DifferentialMotionMagicDutyCycle",
                  List.of(ProfiledPositionDutyCycle.class, PositionDutyCycle.class)),
              Map.entry(
                  "DifferentialMotionMagicVoltage",
                  List.of(ProfiledPositionVoltage.class, PositionVoltage.class)),
              Map.entry(
                  "DifferentialMotionMagicExpoDutyCycle",
                  List.of(ExponentialProfiledPositionDutyCycle.class, PositionDutyCycle.class)),
              Map.entry(
                  "DifferentialMotionMagicExpoVoltage",
                  List.of(ExponentialProfiledPositionVoltage.class, PositionVoltage.class)),
              Map.entry(
                  "DifferentialMotionMagicVelocityDutyCycle",
                  List.of(ProfiledVelocityDutyCycle.class, PositionDutyCycle.class)),
              Map.entry(
                  "DifferentialMotionMagicVelocityVoltage",
                  List.of(ProfiledVelocityVoltage.class, PositionVoltage.class)));

  private static Map<String, Class<? extends MotorRequest>> singleRequests() {
    Map<String, Class<? extends MotorRequest>> map = new TreeMap<>();
    map.put("DutyCycleOut", DutyCycleOutput.class);
    map.put("VoltageOut", VoltageOutput.class);
    map.put("TorqueCurrentFOC", CurrentOutput.class);
    map.put("PositionDutyCycle", PositionDutyCycle.class);
    map.put("PositionVoltage", PositionVoltage.class);
    map.put("PositionTorqueCurrentFOC", PositionCurrent.class);
    map.put("MotionMagicDutyCycle", ProfiledPositionDutyCycle.class);
    map.put("MotionMagicVoltage", ProfiledPositionVoltage.class);
    map.put("MotionMagicTorqueCurrentFOC", ProfiledPositionCurrent.class);
    map.put("MotionMagicExpoDutyCycle", ExponentialProfiledPositionDutyCycle.class);
    map.put("MotionMagicExpoVoltage", ExponentialProfiledPositionVoltage.class);
    map.put("MotionMagicExpoTorqueCurrentFOC", ExponentialProfiledPositionCurrent.class);
    map.put("DynamicMotionMagicDutyCycle", DynamicProfiledPositionDutyCycle.class);
    map.put("DynamicMotionMagicVoltage", DynamicProfiledPositionVoltage.class);
    map.put("DynamicMotionMagicTorqueCurrentFOC", DynamicProfiledPositionCurrent.class);
    map.put("DynamicMotionMagicExpoDutyCycle", DynamicExponentialProfiledPositionDutyCycle.class);
    map.put("DynamicMotionMagicExpoVoltage", DynamicExponentialProfiledPositionVoltage.class);
    map.put(
        "DynamicMotionMagicExpoTorqueCurrentFOC", DynamicExponentialProfiledPositionCurrent.class);
    map.put("VelocityDutyCycle", VelocityDutyCycle.class);
    map.put("VelocityVoltage", VelocityVoltage.class);
    map.put("VelocityTorqueCurrentFOC", VelocityCurrent.class);
    map.put("MotionMagicVelocityDutyCycle", ProfiledVelocityDutyCycle.class);
    map.put("MotionMagicVelocityVoltage", ProfiledVelocityVoltage.class);
    map.put("MotionMagicVelocityTorqueCurrentFOC", ProfiledVelocityCurrent.class);
    map.put("Follower", Follow.class);
    map.put("StrictFollower", StrictFollow.class);
    map.put("DifferentialFollower", DifferentialFollow.class);
    map.put("DifferentialStrictFollower", DifferentialStrictFollow.class);
    map.put("NeutralOut", Neutral.class);
    map.put("CoastOut", Coast.class);
    map.put("StaticBrake", StaticBrake.class);
    map.put("MusicTone", Tone.class);
    return Collections.unmodifiableMap(map);
  }

  private static Set<Class<?>> phoenixRequests() {
    return IOTestSupport.controlRequests(
        CoreTalonFX.class, Set.of(ControlRequest.class, EmptyControl.class));
  }

  /** Every (average, difference) pair a Phoenix6 differential request can express. */
  private static Set<List<Class<?>>> phoenixDifferentialPairs() throws NoSuchFieldException {
    Set<List<Class<?>>> pairs = new HashSet<>();
    for (Class<?> request : phoenixRequests()) {
      if (request.getPackageName().equals(COMPOUND_PACKAGE)) {
        Class<?> average = TALONFX_SINGLE_REQUESTS.get(fieldType(request, "AverageRequest"));
        Class<?> difference =
            TALONFX_SINGLE_REQUESTS.get(fieldType(request, "DifferentialRequest"));
        assertNotNull(average, request.getSimpleName() + " average has no mapping");
        assertNotNull(difference, request.getSimpleName() + " difference has no mapping");
        pairs.add(List.of(average, difference));
      }
    }
    for (List<Class<? extends AxisRequest>> pair : TALONFX_STANDALONE_DIFFERENTIALS.values()) {
      pairs.add(List.of(pair.get(0), pair.get(1)));
    }
    return pairs;
  }

  private static String fieldType(Class<?> request, String field) throws NoSuchFieldException {
    return request.getField(field).getType().getSimpleName();
  }

  @Test
  public void testEveryTalonFXRequestIsMapped() {
    List<String> unmapped = new ArrayList<>();
    int compound = 0;
    for (Class<?> request : phoenixRequests()) {
      String name = request.getSimpleName();
      if (request.getPackageName().equals(COMPOUND_PACKAGE)) {
        compound++;
      } else if (!TALONFX_SINGLE_REQUESTS.containsKey(name)
          && !TALONFX_STANDALONE_DIFFERENTIALS.containsKey(name)) {
        unmapped.add(name);
      }
    }
    assertTrue(unmapped.isEmpty(), "TalonFX requests with no MotorRequest mapping: " + unmapped);
    assertEquals(54, compound, "compound differential requests");
  }

  @Test
  public void testMappingsNameOnlyRealTalonFXRequests() {
    Set<String> phoenix = new TreeSet<>();
    for (Class<?> request : phoenixRequests()) {
      phoenix.add(request.getSimpleName());
    }
    for (String name : TALONFX_SINGLE_REQUESTS.keySet()) {
      assertTrue(phoenix.contains(name), name);
    }
    for (String name : TALONFX_STANDALONE_DIFFERENTIALS.keySet()) {
      assertTrue(phoenix.contains(name), name);
    }
  }

  @Test
  public void testEveryRecordIsUsed() {
    Set<Class<?>> used = new HashSet<>(TALONFX_SINGLE_REQUESTS.values());
    used.add(Differential.class);
    assertEquals(
        new HashSet<>(IOTestSupport.leafRecords(MotorRequest.class)),
        used,
        "every MotorRequest record maps to a TalonFX request");
  }

  @Test
  public void testDifferentialAcceptsExactlyThePhoenixPairs() throws ReflectiveOperationException {
    Set<List<Class<?>>> valid = phoenixDifferentialPairs();
    List<Class<?>> axes = IOTestSupport.leafRecords(AxisRequest.class);
    assertEquals(54, valid.size(), "distinct differential pairs");
    for (Class<?> average : axes) {
      for (Class<?> difference : axes) {
        AxisRequest a = (AxisRequest) IOTestSupport.sampleRecord(average);
        AxisRequest d = (AxisRequest) IOTestSupport.sampleRecord(difference);
        String pair = average.getSimpleName() + " / " + difference.getSimpleName();
        if (valid.contains(List.of(average, difference))) {
          assertDoesNotThrow(() -> new Differential(a, d), pair);
        } else {
          assertThrows(IllegalArgumentException.class, () -> new Differential(a, d), pair);
        }
      }
    }
  }

  /** Fails to compile if a {@link MotorRequest} record is added without a case here. */
  private static String describe(MotorRequest request) {
    return switch (request) {
      case DutyCycleOutput _ -> "DutyCycleOutput";
      case VoltageOutput _ -> "VoltageOutput";
      case CurrentOutput _ -> "CurrentOutput";
      case PositionDutyCycle _ -> "PositionDutyCycle";
      case PositionVoltage _ -> "PositionVoltage";
      case PositionCurrent _ -> "PositionCurrent";
      case ProfiledPositionDutyCycle _ -> "ProfiledPositionDutyCycle";
      case ProfiledPositionVoltage _ -> "ProfiledPositionVoltage";
      case ProfiledPositionCurrent _ -> "ProfiledPositionCurrent";
      case ExponentialProfiledPositionDutyCycle _ -> "ExponentialProfiledPositionDutyCycle";
      case ExponentialProfiledPositionVoltage _ -> "ExponentialProfiledPositionVoltage";
      case ExponentialProfiledPositionCurrent _ -> "ExponentialProfiledPositionCurrent";
      case DynamicProfiledPositionDutyCycle _ -> "DynamicProfiledPositionDutyCycle";
      case DynamicProfiledPositionVoltage _ -> "DynamicProfiledPositionVoltage";
      case DynamicProfiledPositionCurrent _ -> "DynamicProfiledPositionCurrent";
      case DynamicExponentialProfiledPositionDutyCycle _ ->
          "DynamicExponentialProfiledPositionDutyCycle";
      case DynamicExponentialProfiledPositionVoltage _ ->
          "DynamicExponentialProfiledPositionVoltage";
      case DynamicExponentialProfiledPositionCurrent _ ->
          "DynamicExponentialProfiledPositionCurrent";
      case VelocityDutyCycle _ -> "VelocityDutyCycle";
      case VelocityVoltage _ -> "VelocityVoltage";
      case VelocityCurrent _ -> "VelocityCurrent";
      case ProfiledVelocityDutyCycle _ -> "ProfiledVelocityDutyCycle";
      case ProfiledVelocityVoltage _ -> "ProfiledVelocityVoltage";
      case ProfiledVelocityCurrent _ -> "ProfiledVelocityCurrent";
      case Differential _ -> "Differential";
      case Follow _ -> "Follow";
      case StrictFollow _ -> "StrictFollow";
      case DifferentialFollow _ -> "DifferentialFollow";
      case DifferentialStrictFollow _ -> "DifferentialStrictFollow";
      case Neutral _ -> "Neutral";
      case Coast _ -> "Coast";
      case StaticBrake _ -> "StaticBrake";
      case Tone _ -> "Tone";
    };
  }

  @Test
  public void testExhaustiveSwitchCoversEveryRecord() throws ReflectiveOperationException {
    for (Class<?> record : IOTestSupport.leafRecords(MotorRequest.class)) {
      MotorRequest request = (MotorRequest) IOTestSupport.sampleRecord(record);
      assertEquals(record.getSimpleName(), describe(request));
    }
  }
}
