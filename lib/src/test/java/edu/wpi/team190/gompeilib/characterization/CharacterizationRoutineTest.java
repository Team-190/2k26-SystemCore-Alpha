package edu.wpi.team190.gompeilib.characterization;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig.Leg;
import edu.wpi.team190.gompeilib.characterization.CharacterizationRoutine.EndReason;
import edu.wpi.team190.gompeilib.characterization.CharacterizationRoutine.MechanismTarget;
import edu.wpi.team190.gompeilib.characterization.CharacterizationRoutine.Target;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Coefficient;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Result;
import edu.wpi.team190.gompeilib.characterization.FeedforwardFit.Sample;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GravityType;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OutputType;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.sim.MotorIOSim;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import edu.wpi.team190.gompeilib.io.physics.PlantMechanismSim;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControl;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DimensionlessUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Distance;

public class CharacterizationRoutineTest extends GompeiLibTestBase {
  private static final String STATE = "Test/Mechanism/Characterization/State";
  private static final String END_REASON = "Test/Mechanism/Characterization/EndReason";
  private static final Function<Measure<VoltageUnit>, MotorRequest> VOLTS =
      v -> new OpenLoopRequest.VoltageOutput(Volts.of(v.in(Volts)), true, RequestOptions.DEFAULT);
  private static final Function<Measure<CurrentUnit>, MotorRequest> AMPS =
      a -> new OpenLoopRequest.CurrentOutput(Amps.of(a.in(Amps)));

  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private MechanismCore core;
  private final AtomicReference<Distance> position = new AtomicReference<>(Meters.of(0.5));
  private CharacterizationConstraint<DistanceUnit> constraint;

  @Override
  protected boolean isTuningMode() {
    return true;
  }

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    SimHooks.pauseTiming();
    motors = new TestMotors();
    core = motors.core();
    constraint =
        new CharacterizationConstraint<>(
            position::get, Meters.of(0), Meters.of(1), () -> Meters.of(0.1));
  }

  @AfterEach
  public void tearDown() {
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    SimHooks.resumeTiming();
    logger.close();
  }

  private static CharacterizationConfig.CharacterizationConfigBuilder<VoltageUnit> config() {
    return CharacterizationConfig.voltage()
        .withRampRate(CustomUnits.voltsPerSecond.ofNative(2))
        .withStepOutput(Volts.of(6))
        .withQuasistaticTimeout(Seconds.of(4))
        .withDynamicTimeout(Seconds.of(1))
        .withSettleTime(Seconds.of(0.5));
  }

  private static <U extends Unit> MechanismTarget<U> target(
      MechanismCore core,
      Function<Measure<U>, MotorRequest> request,
      Optional<CharacterizationConstraint<?>> constraint) {
    return new MechanismTarget<>(core, request, constraint, Optional.empty(), Optional.empty());
  }

  private CharacterizationRoutine<VoltageUnit> routine(
      CharacterizationConfig<VoltageUnit> config,
      Optional<CharacterizationConstraint<?>> constraint) {
    return new CharacterizationRoutine<>(
        config, motors.subsystem, List.of(target(core, VOLTS, constraint)));
  }

  private static OpenLoopRequest.VoltageOutput volts(double volts) {
    return new OpenLoopRequest.VoltageOutput(Volts.of(volts), true, RequestOptions.DEFAULT);
  }

  /** Steps time, then runs one loop of the command, as the scheduler would. */
  private static boolean loop(Command command, double seconds) {
    SimHooks.stepTiming(seconds);
    command.execute();
    return command.isFinished();
  }

  /** A target that reports samples from a known plant, one per call. */
  private static final class SyntheticTarget implements Target<VoltageUnit> {
    private final double kS;
    private final double kV;
    private final double kA;
    private final double kG;
    private final Optional<GravityType> gravity;
    private int calls = 0;

    SyntheticTarget(double kS, double kV, double kA, double kG, Optional<GravityType> gravity) {
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
      this.kG = kG;
      this.gravity = gravity;
    }

    @Override
    public String topic() {
      return "Test/Synthetic";
    }

    @Override
    public void idle() {}

    @Override
    public void drive(Measure<VoltageUnit> output) {}

    @Override
    public Sample sample(OutputType effort) {
      calls++;
      double velocity = 3 * Math.sin(calls * 0.37) + (calls % 2 == 0 ? 0.5 : -0.5);
      double acceleration = 4 * Math.cos(calls * 0.91);
      double angle = 0.8 * Math.sin(calls * 0.13);
      double load = gravity.isPresent() ? kG * Math.cos(angle) : 0.0;
      double effortValue = kS * Math.signum(velocity) + kV * velocity + kA * acceleration + load;
      return new Sample(effortValue, angle, velocity, acceleration, velocity * kV);
    }

    @Override
    public boolean atLimit(Leg leg) {
      return false;
    }

    @Override
    public Optional<GravityType> gravity() {
      return gravity;
    }

    @Override
    public Optional<Distance> drumRadius() {
      return Optional.empty();
    }
  }

  @Test
  public void testQuasistaticRampsTheOutputEachWay() {
    CharacterizationRoutine<VoltageUnit> routine = routine(config().build(), Optional.empty());

    Command forward = routine.leg(Leg.QUASISTATIC_FORWARD);
    forward.initialize();
    assertFalse(loop(forward, 0.5));
    verify(motors.motor).setControl(volts(1.0));
    assertFalse(loop(forward, 0.5));
    verify(motors.motor).setControl(volts(2.0));
    logger.verify(() -> Logger.recordOutput(STATE, "quasistatic-forward"), times(2));

    Command reverse = routine.leg(Leg.QUASISTATIC_REVERSE);
    reverse.initialize();
    assertFalse(loop(reverse, 0.25));
    verify(motors.motor).setControl(volts(-0.5));
    logger.verify(() -> Logger.recordOutput(STATE, "quasistatic-reverse"));
  }

  @Test
  public void testDynamicStepsTheOutputEachWay() {
    CharacterizationRoutine<VoltageUnit> routine = routine(config().build(), Optional.empty());

    Command forward = routine.leg(Leg.DYNAMIC_FORWARD);
    forward.initialize();
    assertFalse(loop(forward, 0.02));
    verify(motors.motor).setControl(volts(6.0));
    logger.verify(() -> Logger.recordOutput(STATE, "dynamic-forward"));

    Command reverse = routine.leg(Leg.DYNAMIC_REVERSE);
    reverse.initialize();
    assertFalse(loop(reverse, 0.02));
    verify(motors.motor).setControl(volts(-6.0));
    logger.verify(() -> Logger.recordOutput(STATE, "dynamic-reverse"));
  }

  @Test
  public void testUnconstrainedLegsRunToTheirTimeout() {
    CharacterizationRoutine<VoltageUnit> routine = routine(config().build(), Optional.empty());
    Command leg = routine.leg(Leg.QUASISTATIC_FORWARD);
    leg.initialize();
    assertFalse(loop(leg, 3.9));
    assertTrue(loop(leg, 0.1));
    leg.end(false);
    verify(motors.motor).setControl(volts(0));
    logger.verify(() -> Logger.recordOutput(STATE, "none"));
    logger.verify(() -> Logger.recordOutput(END_REASON, "TIMEOUT"));

    Command dynamic = routine.leg(Leg.DYNAMIC_FORWARD);
    dynamic.initialize();
    assertFalse(loop(dynamic, 0.9));
    assertTrue(loop(dynamic, 0.1));
  }

  @Test
  public void testStopsAtTheLimitInItsDirectionOnly() {
    CharacterizationRoutine<VoltageUnit> routine =
        routine(config().build(), Optional.of(constraint));

    // Starting next to the lower limit doesn't stop a forward leg.
    position.set(Meters.of(0.05));
    Command forward = routine.leg(Leg.DYNAMIC_FORWARD);
    forward.initialize();
    assertFalse(loop(forward, 0.02));
    position.set(Meters.of(0.95));
    assertTrue(forward.isFinished());
    forward.end(false);
    verify(motors.motor).setControl(volts(0));
    logger.verify(() -> Logger.recordOutput(END_REASON, "LIMIT"));

    Command reverse = routine.leg(Leg.DYNAMIC_REVERSE);
    reverse.initialize();
    assertFalse(loop(reverse, 0.02));
    position.set(Meters.of(0.08));
    assertTrue(reverse.isFinished());
  }

  @Test
  public void testFirstConstrainedTargetToTripEndsTheLeg() {
    TestMotors otherMotors = new TestMotors();
    MechanismCore other = new MechanismCore(otherMotors.group, otherMotors.subsystem, "Other");
    AtomicReference<Distance> otherPosition = new AtomicReference<>(Meters.of(0.5));
    CharacterizationConstraint<DistanceUnit> otherConstraint =
        new CharacterizationConstraint<>(
            otherPosition::get, Meters.of(0), Meters.of(1), () -> Meters.of(0.1));
    CharacterizationRoutine<VoltageUnit> routine =
        new CharacterizationRoutine<>(
            config().build(),
            motors.subsystem,
            List.of(
                target(core, VOLTS, Optional.of(constraint)),
                target(other, VOLTS, Optional.of(otherConstraint))));

    Command leg = routine.leg(Leg.QUASISTATIC_FORWARD);
    leg.initialize();
    assertFalse(loop(leg, 0.5));
    verify(motors.motor).setControl(volts(1.0));
    verify(otherMotors.motor).setControl(volts(1.0));
    assertEquals(1, routine.samples(0).size());
    assertEquals(1, routine.samples(1).size());

    otherPosition.set(Meters.of(0.95));
    assertTrue(leg.isFinished());
    leg.end(false);
    verify(motors.motor).setControl(volts(0));
    verify(otherMotors.motor).setControl(volts(0));
    logger.verify(() -> Logger.recordOutput(END_REASON, "LIMIT"));
    logger.verify(() -> Logger.recordOutput("Test/Other/Characterization/EndReason", "LIMIT"));
  }

  @Test
  public void testSkipsALegThatStartsInsideTheMargin() {
    position.set(Meters.of(0.95));
    CharacterizationRoutine<VoltageUnit> routine =
        routine(config().build(), Optional.of(constraint));
    Command leg = routine.leg(Leg.DYNAMIC_FORWARD);
    leg.initialize();
    assertTrue(leg.isFinished());
    leg.execute();
    verify(motors.motor, never()).setControl(volts(6.0));
    assertTrue(routine.samples(0).isEmpty());
    leg.end(false);
    verify(motors.motor).setControl(volts(0));
    logger.verify(() -> Logger.recordOutput(END_REASON, "SKIPPED"));
    logger.verify(() -> Logger.recordOutput(STATE, "dynamic-forward"), never());
  }

  @Test
  public void testCancellingZeroesTheOutput() {
    CharacterizationRoutine<VoltageUnit> routine = routine(config().build(), Optional.empty());
    Command leg = routine.leg(Leg.DYNAMIC_REVERSE);
    leg.initialize();
    assertFalse(loop(leg, 0.02));
    leg.end(true);
    verify(motors.motor).setControl(volts(0));
    logger.verify(() -> Logger.recordOutput(END_REASON, "CANCELLED"));
  }

  @Test
  public void testDrivesTorqueCurrent() {
    CharacterizationConfig<CurrentUnit> config =
        CharacterizationConfig.current()
            .withRampRate(CustomUnits.ampsPerSecond.ofNative(4))
            .withStepOutput(Amps.of(30))
            .withQuasistaticTimeout(Seconds.of(5))
            .withDynamicTimeout(Seconds.of(5))
            .build();
    CharacterizationRoutine<CurrentUnit> routine =
        new CharacterizationRoutine<>(
            config,
            motors.subsystem,
            List.of(
                target(
                    core,
                    a -> new OpenLoopRequest.CurrentOutput(Amps.of(a.in(Amps))),
                    Optional.empty())));

    Command quasistatic = routine.leg(Leg.QUASISTATIC_REVERSE);
    quasistatic.initialize();
    loop(quasistatic, 0.5);
    verify(motors.motor).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(-2)));

    Command dynamic = routine.leg(Leg.DYNAMIC_FORWARD);
    dynamic.initialize();
    loop(dynamic, 0.02);
    verify(motors.motor).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(30)));
    dynamic.end(true);
    verify(motors.motor).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(0)));
  }

  @Test
  public void testReadsTunablesWhenALegStarts() {
    CharacterizationConfig<VoltageUnit> config = config().build();
    CharacterizationRoutine<VoltageUnit> routine = routine(config, Optional.empty());

    Command first = routine.leg(Leg.DYNAMIC_FORWARD);
    first.initialize();
    config.stepOutput().set(Volts.of(3));
    config.dynamicTimeout().set(Seconds.of(0.2));
    loop(first, 0.5);
    verify(motors.motor).setControl(volts(6.0));
    assertFalse(first.isFinished());

    Command second = routine.leg(Leg.DYNAMIC_FORWARD);
    second.initialize();
    assertFalse(loop(second, 0.1));
    verify(motors.motor).setControl(volts(3.0));
    assertTrue(loop(second, 0.1));

    config.rampRate().set(CustomUnits.voltsPerSecond.ofNative(8));
    Command ramp = routine.leg(Leg.QUASISTATIC_FORWARD);
    ramp.initialize();
    loop(ramp, 0.5);
    verify(motors.motor).setControl(volts(4.0));
  }

  @Test
  public void testRunsTheConfiguredLegsInOrderWithSettleTimeThenReports() {
    CharacterizationConfig<VoltageUnit> config =
        config()
            .withQuasistaticTimeout(Seconds.of(0.1))
            .withDynamicTimeout(Seconds.of(0.1))
            .withLegs(Set.of(Leg.DYNAMIC_FORWARD, Leg.QUASISTATIC_FORWARD))
            .build();
    VoltageControl voltage = new VoltageControl(core, Volts.of(1), true);
    voltage.setGoal(Volts.of(2));
    CharacterizationRoutine<VoltageUnit> routine = routine(config, Optional.empty());

    Command command = routine.command();
    assertEquals("Characterization", command.getName());
    assertTrue(command.getRequirements().contains(motors.subsystem));
    command.initialize();
    assertEquals(MechanismCore.IDLE, core.getMode());

    // The idle step finishes, then quasistatic forward runs until it times out.
    for (int i = 0; i < 8; i++) {
      loop(command, 0.02);
    }
    logger.verify(() -> Logger.recordOutput(STATE, "quasistatic-forward"), atLeastOnce());
    logger.verify(() -> Logger.recordOutput(END_REASON, "TIMEOUT"));

    // Settling: dynamic forward hasn't started yet.
    for (int i = 0; i < 10; i++) {
      loop(command, 0.02);
    }
    logger.verify(() -> Logger.recordOutput(STATE, "dynamic-forward"), never());

    boolean finished = false;
    for (int i = 0; i < 40 && !finished; i++) {
      finished = loop(command, 0.02);
    }
    assertTrue(finished);
    command.end(false);
    logger.verify(() -> Logger.recordOutput(STATE, "dynamic-forward"), atLeastOnce());
    logger.verify(() -> Logger.recordOutput(END_REASON, "TIMEOUT"), times(2));
    logger.verify(() -> Logger.recordOutput(STATE, "quasistatic-reverse"), never());
    logger.verify(() -> Logger.recordOutput(STATE, "dynamic-reverse"), never());
    // The mechanism never moved, so the report explains why there's no fit.
    assertTrue(routine.report().contains("FAILED: Need at least"));
  }

  @Test
  public void testLegNamesAndRequirements() {
    CharacterizationRoutine<VoltageUnit> routine = routine(config().build(), Optional.empty());
    Command leg = routine.leg(Leg.QUASISTATIC_REVERSE);
    assertEquals("characterize-quasistatic-reverse-Test", leg.getName());
    assertEquals(Set.of(motors.subsystem), leg.getRequirements());
  }

  @Test
  public void shouldRejectBadArguments() {
    CharacterizationConfig<VoltageUnit> config = config().build();
    Target<VoltageUnit> target = target(core, VOLTS, Optional.empty());
    assertThrows(
        IllegalArgumentException.class,
        () -> new CharacterizationRoutine<>(config, motors.subsystem, List.of()));
    assertThrows(
        NullPointerException.class,
        () -> new CharacterizationRoutine<>(null, motors.subsystem, List.of(target)));
    assertThrows(
        NullPointerException.class,
        () -> new CharacterizationRoutine<>(config, null, List.of(target)));
    CharacterizationConfig<DimensionlessUnit> dutyCycle =
        CharacterizationConfig.<DimensionlessUnit>builder()
            .withOutputUnit(Percent)
            .withRampRate(Percent.per(Second).ofNative(1))
            .withStepOutput(Percent.of(10))
            .withQuasistaticTimeout(Seconds.of(1))
            .withDynamicTimeout(Seconds.of(1))
            .build();
    assertThrows(
        IllegalArgumentException.class,
        () ->
            new CharacterizationRoutine<DimensionlessUnit>(
                dutyCycle,
                motors.subsystem,
                List.of(target(core, p -> volts(0), Optional.empty()))));
  }

  @Test
  public void shouldRequireEveryTargetPart() {
    Optional<CharacterizationConstraint<?>> none = Optional.empty();
    assertThrows(
        NullPointerException.class,
        () -> new MechanismTarget<>(null, VOLTS, none, Optional.empty(), Optional.empty()));
    assertThrows(
        NullPointerException.class,
        () ->
            new MechanismTarget<VoltageUnit>(core, null, none, Optional.empty(), Optional.empty()));
    assertThrows(
        NullPointerException.class,
        () -> new MechanismTarget<>(core, VOLTS, null, Optional.empty(), Optional.empty()));
    assertThrows(
        NullPointerException.class,
        () -> new MechanismTarget<>(core, VOLTS, none, null, Optional.empty()));
    assertThrows(
        NullPointerException.class,
        () -> new MechanismTarget<>(core, VOLTS, none, Optional.empty(), null));
  }

  @Test
  public void testMechanismTargetReadsItsLeader() {
    MechanismTarget<VoltageUnit> target = target(core, VOLTS, Optional.empty());
    assertEquals("Test/Mechanism", target.topic());
    target.idle();
    assertEquals(MechanismCore.IDLE, core.getMode());

    motors.read(
        t -> {
          t.appliedVoltage = Volts.of(4);
          t.torqueCurrent = Amps.of(12);
          t.position = Radians.of(1);
          t.velocity = RadiansPerSecond.of(2);
          t.acceleration = RadiansPerSecondPerSecond.of(3);
          t.rotorVelocity = RadiansPerSecond.of(20);
          t.motorKV = RadiansPerSecond.per(Volt).ofNative(40);
        });
    assertEquals(new Sample(4, 1, 2, 3, 0.5), target.sample(OutputType.VOLTAGE));
    assertEquals(12, target.sample(OutputType.CURRENT).effort());
  }

  @Test
  public void testSampleLeavesBackEmfUnknownWithoutAMotorModel() {
    MotorTelemetry telemetry = new MotorTelemetry();
    telemetry.velocity = RadiansPerSecond.of(2);
    assertTrue(
        Double.isNaN(CharacterizationRoutine.sample(telemetry, OutputType.VOLTAGE).backEmf()));
  }

  @Test
  public void testReportPrintsThePlantAndGains() {
    SyntheticTarget target = new SyntheticTarget(0.3, 0.2, 0.05, 0.0, Optional.empty());
    CharacterizationRoutine<VoltageUnit> routine =
        new CharacterizationRoutine<>(config().build(), motors.subsystem, List.of(target));
    Command leg = routine.leg(Leg.DYNAMIC_FORWARD);
    leg.initialize();
    for (int i = 0; i < 40; i++) {
      leg.execute();
    }

    Result result = routine.fit(0);
    assertEquals(0.3, result.kS().value(), 1e-9);
    assertEquals(0.2, result.kV().value(), 1e-9);
    assertEquals(0.05, result.kA().value(), 1e-9);

    String report = routine.report();
    assertTrue(report.startsWith("********** Characterization: Test/Synthetic **********\n"));
    assertTrue(report.contains("PlantCharacterization.voltage()\n"));
    assertTrue(report.contains("    .withKS(Volts.of(0.300000))\n"));
    assertTrue(
        report.contains(
            "    .withKV(Volts.per(RotationsPerSecond).ofNative("
                + String.format(java.util.Locale.ROOT, "%.6g", 0.2 * 2 * Math.PI)
                + "))\n"));
    logger.verify(() -> Logger.recordOutput(eq("Test/Synthetic/Characterization/kS"), anyDouble()));
    logger.verify(() -> Logger.recordOutput(eq("Test/Synthetic/Characterization/Samples"), eq(40)));
  }

  @Test
  public void testFormatsCurrentLinearAndGravityFits() {
    Result result =
        new Result(
            new Coefficient(2.0, 0.01),
            new Coefficient(0.5, 0.01),
            new Coefficient(-0.1, 0.01),
            new Coefficient(1.5, 0.01),
            Optional.of(GravityType.CONSTANT),
            0.99,
            0.25,
            100,
            OptionalDouble.empty());

    String text =
        CharacterizationRoutine.format(result, OutputType.CURRENT, Optional.of(Meters.of(0.5)));

    assertTrue(text.startsWith("Samples: 100 | R²: 0.9900 | RMSE: 0.2500 A\n"));
    assertTrue(text.contains("WARNING: kA is negative\n"));
    assertTrue(text.contains("PlantCharacterization.current()\n"));
    assertTrue(text.contains("    .withKG(Amps.of(1.50000))\n"));
    assertTrue(text.contains("    .withGravityType(GravityType.CONSTANT)\n"));
    assertTrue(text.contains("    .withKV(Amps.per(MetersPerSecond).ofNative(1.00000))\n"));
    assertTrue(
        text.contains("    .withKA(Amps.per(MetersPerSecondPerSecond).ofNative(-0.200000))\n"));

    String angular =
        CharacterizationRoutine.format(
            new Result(
                result.kS(),
                result.kV(),
                new Coefficient(0.1, 0.01),
                Coefficient.NONE,
                Optional.empty(),
                0.99,
                0.25,
                100,
                OptionalDouble.empty()),
            OutputType.VOLTAGE,
            Optional.empty());
    assertFalse(angular.contains("WARNING"));
    assertFalse(angular.contains("withKG"));
    assertFalse(angular.contains("withGravityType"));
    assertTrue(angular.contains("Gains:\n    .withKS(Volts.of(2.00000))\n"));
  }

  /**
   * Characterizes a simulated mechanism with a known plant and checks the fit recovers it: the sim
   * and the characterization agree, so gains tuned against one hold against the other.
   */
  private static <U extends Unit> Result roundTrip(
      PlantCharacterization<?> plant,
      CharacterizationConfig<U> config,
      Function<Measure<U>, MotorRequest> request) {
    DCMotor gearbox = DCMotor.getKrakenX60(1);
    PlantMechanismSim mechanism = new PlantMechanismSim(gearbox, 5.0, plant);
    MotorControllerGroup group =
        new MotorControllerGroup(
            MotorConfig.builder()
                .withCanId(1)
                .withBus(CANBusId.named("rio"))
                .withGearRatio(5.0)
                .withCurrentLimits(new CurrentLimits(120, 120))
                .build(),
            MotorIOSim.factory(mechanism));
    TestMotors owner = new TestMotors();
    MechanismCore core = new MechanismCore(group, owner.subsystem, "RoundTrip");
    CharacterizationRoutine<U> routine =
        new CharacterizationRoutine<>(
            config, owner.subsystem, List.of(target(core, request, Optional.empty())));

    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    Command command = routine.command();
    command.initialize();
    boolean finished = false;
    for (int i = 0; i < 2000 && !finished; i++) {
      SimHooks.stepTiming(0.02);
      core.periodic();
      command.execute();
      finished = command.isFinished();
    }
    assertTrue(finished);
    command.end(false);
    return routine.fit(0);
  }

  @Test
  public void testRecoversAVoltagePlantFromSimulation() {
    PlantCharacterization<VoltageUnit> plant =
        PlantCharacterization.voltage()
            .withKS(Volts.of(0.3))
            .withKV(Volts.per(RadiansPerSecond).ofNative(0.2))
            .withKA(Volts.per(RadiansPerSecondPerSecond).ofNative(0.05))
            .build();

    Result result =
        roundTrip(
            plant,
            config()
                .withRampRate(CustomUnits.voltsPerSecond.ofNative(1))
                .withQuasistaticTimeout(Seconds.of(4))
                .withDynamicTimeout(Seconds.of(1))
                .build(),
            VOLTS);

    assertEquals(0.3, result.kS().value(), 0.03);
    assertEquals(0.2, result.kV().value(), 0.01);
    assertEquals(0.05, result.kA().value(), 0.005);
    assertTrue(result.rSquared() > 0.99);
    assertTrue(result.warnings(true).isEmpty(), result.warnings(true).toString());
  }

  @Test
  public void testRecoversACurrentPlantFromSimulation() {
    PlantCharacterization<CurrentUnit> plant =
        PlantCharacterization.current()
            .withKS(Amps.of(2.0))
            .withKV(Amps.per(RadiansPerSecond).ofNative(0.1))
            .withKA(Amps.per(RadiansPerSecondPerSecond).ofNative(0.5))
            .build();

    Result result =
        roundTrip(
            plant,
            CharacterizationConfig.current()
                .withRampRate(CustomUnits.ampsPerSecond.ofNative(4))
                .withStepOutput(Amps.of(20))
                .withQuasistaticTimeout(Seconds.of(4))
                .withDynamicTimeout(Seconds.of(1))
                .withSettleTime(Seconds.of(1))
                .build(),
            AMPS);

    assertEquals(2.0, result.kS().value(), 0.2);
    assertEquals(0.1, result.kV().value(), 0.01);
    assertEquals(0.5, result.kA().value(), 0.05);
    assertTrue(result.rSquared() > 0.99);
  }

  @Test
  public void testEndReasons() {
    assertArrayEquals(
        new EndReason[] {
          EndReason.LIMIT, EndReason.TIMEOUT, EndReason.SKIPPED, EndReason.CANCELLED
        },
        EndReason.values());
    assertEquals(EndReason.SKIPPED, EndReason.valueOf("SKIPPED"));
  }

  @Test
  public void testCollectsSamplesAcrossLegsUntilTheCommandRestarts() {
    SyntheticTarget target = new SyntheticTarget(0.3, 0.2, 0.05, 0.0, Optional.empty());
    CharacterizationRoutine<VoltageUnit> routine =
        new CharacterizationRoutine<>(
            config().withLegs(Set.of(Leg.DYNAMIC_FORWARD)).build(),
            motors.subsystem,
            List.of(target));
    Command leg = routine.leg(Leg.DYNAMIC_FORWARD);
    leg.initialize();
    leg.execute();
    assertEquals(1, routine.samples(0).size());

    Command command = routine.command();
    command.initialize();
    assertTrue(routine.samples(0).isEmpty());
    List<Sample> copy = new ArrayList<>(routine.samples(0));
    assertTrue(copy.isEmpty());
  }
}
