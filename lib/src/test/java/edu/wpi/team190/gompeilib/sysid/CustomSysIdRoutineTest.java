package edu.wpi.team190.gompeilib.sysid;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.math.CustomUnits;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import edu.wpi.team190.gompeilib.robot.mechanisms.VoltageControl;
import edu.wpi.team190.gompeilib.sysid.CustomSysIdRoutine.EndReason;
import edu.wpi.team190.gompeilib.sysid.CustomSysIdRoutine.Target;
import edu.wpi.team190.gompeilib.sysid.SysIdConfig.Leg;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.simulation.SimHooks;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Distance;

public class CustomSysIdRoutineTest extends GompeiLibTestBase {
  private static final String STATE = "Test/Mechanism/SysId/State";
  private static final String END_REASON = "Test/Mechanism/SysId/EndReason";
  private static final Function<Measure<VoltageUnit>, MotorRequest> VOLTS =
      v -> new OpenLoopRequest.VoltageOutput(Volts.of(v.in(Volts)), true, RequestOptions.DEFAULT);

  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private MechanismCore core;
  private final AtomicReference<Distance> position = new AtomicReference<>(Meters.of(0.5));
  private SysIdConstraint<DistanceUnit> constraint;

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
        new SysIdConstraint<>(position::get, Meters.of(0), Meters.of(1), () -> Meters.of(0.1));
  }

  @AfterEach
  public void tearDown() {
    SimHooks.resumeTiming();
    logger.close();
  }

  private static SysIdConfig.SysIdConfigBuilder<VoltageUnit> config() {
    return SysIdConfig.voltage()
        .withRampRate(CustomUnits.voltsPerSecond.ofNative(2))
        .withStepOutput(Volts.of(6))
        .withQuasistaticTimeout(Seconds.of(4))
        .withDynamicTimeout(Seconds.of(1))
        .withSettleTime(Seconds.of(0.5));
  }

  private CustomSysIdRoutine<VoltageUnit> routine(
      SysIdConfig<VoltageUnit> config, Optional<SysIdConstraint<?>> constraint) {
    return new CustomSysIdRoutine<>(
        config, motors.subsystem, List.of(new Target<>(core, VOLTS, constraint)));
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

  @Test
  public void testQuasistaticRampsTheOutputEachWay() {
    CustomSysIdRoutine<VoltageUnit> routine = routine(config().build(), Optional.empty());

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
    CustomSysIdRoutine<VoltageUnit> routine = routine(config().build(), Optional.empty());

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
    CustomSysIdRoutine<VoltageUnit> routine = routine(config().build(), Optional.empty());
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
    CustomSysIdRoutine<VoltageUnit> routine = routine(config().build(), Optional.of(constraint));

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
  public void testFirstConstrainedMechanismToTripEndsTheLeg() {
    TestMotors otherMotors = new TestMotors();
    MechanismCore other = new MechanismCore(otherMotors.group, otherMotors.subsystem, "Other");
    AtomicReference<Distance> otherPosition = new AtomicReference<>(Meters.of(0.5));
    SysIdConstraint<DistanceUnit> otherConstraint =
        new SysIdConstraint<>(otherPosition::get, Meters.of(0), Meters.of(1), () -> Meters.of(0.1));
    CustomSysIdRoutine<VoltageUnit> routine =
        new CustomSysIdRoutine<>(
            config().build(),
            motors.subsystem,
            List.of(
                new Target<>(core, VOLTS, Optional.of(constraint)),
                new Target<>(other, VOLTS, Optional.of(otherConstraint))));

    Command leg = routine.leg(Leg.QUASISTATIC_FORWARD);
    leg.initialize();
    assertFalse(loop(leg, 0.5));
    verify(motors.motor).setControl(volts(1.0));
    verify(otherMotors.motor).setControl(volts(1.0));

    otherPosition.set(Meters.of(0.95));
    assertTrue(leg.isFinished());
    leg.end(false);
    verify(motors.motor).setControl(volts(0));
    verify(otherMotors.motor).setControl(volts(0));
    logger.verify(() -> Logger.recordOutput(END_REASON, "LIMIT"));
    logger.verify(() -> Logger.recordOutput("Test/Other/SysId/EndReason", "LIMIT"));
  }

  @Test
  public void testSkipsALegThatStartsInsideTheMargin() {
    position.set(Meters.of(0.95));
    CustomSysIdRoutine<VoltageUnit> routine = routine(config().build(), Optional.of(constraint));
    Command leg = routine.leg(Leg.DYNAMIC_FORWARD);
    leg.initialize();
    assertTrue(leg.isFinished());
    leg.execute();
    verify(motors.motor, never()).setControl(volts(6.0));
    leg.end(false);
    verify(motors.motor).setControl(volts(0));
    logger.verify(() -> Logger.recordOutput(END_REASON, "SKIPPED"));
    logger.verify(() -> Logger.recordOutput(STATE, "dynamic-forward"), never());
  }

  @Test
  public void testCancellingZeroesTheOutput() {
    CustomSysIdRoutine<VoltageUnit> routine = routine(config().build(), Optional.empty());
    Command leg = routine.leg(Leg.DYNAMIC_REVERSE);
    leg.initialize();
    assertFalse(loop(leg, 0.02));
    leg.end(true);
    verify(motors.motor).setControl(volts(0));
    logger.verify(() -> Logger.recordOutput(END_REASON, "CANCELLED"));
  }

  @Test
  public void testDrivesTorqueCurrent() {
    SysIdConfig<CurrentUnit> config =
        SysIdConfig.current()
            .withRampRate(CustomUnits.ampsPerSecond.ofNative(4))
            .withStepOutput(Amps.of(30))
            .withQuasistaticTimeout(Seconds.of(5))
            .withDynamicTimeout(Seconds.of(5))
            .build();
    CustomSysIdRoutine<CurrentUnit> routine =
        new CustomSysIdRoutine<>(
            config,
            motors.subsystem,
            List.of(
                new Target<CurrentUnit>(
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
    SysIdConfig<VoltageUnit> config = config().build();
    CustomSysIdRoutine<VoltageUnit> routine = routine(config, Optional.empty());

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
  public void testRunsTheConfiguredLegsInOrderWithSettleTime() {
    SysIdConfig<VoltageUnit> config =
        config()
            .withQuasistaticTimeout(Seconds.of(0.1))
            .withDynamicTimeout(Seconds.of(0.1))
            .withLegs(Set.of(Leg.DYNAMIC_FORWARD, Leg.QUASISTATIC_FORWARD))
            .build();
    VoltageControl voltage = new VoltageControl(core, Volts.of(1), true);
    voltage.setGoal(Volts.of(2));
    CustomSysIdRoutine<VoltageUnit> routine = routine(config, Optional.empty());

    Command command = routine.command();
    assertEquals("SysId", command.getName());
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
  }

  @Test
  public void testLegNamesAndRequirements() {
    CustomSysIdRoutine<VoltageUnit> routine = routine(config().build(), Optional.empty());
    Command leg = routine.leg(Leg.QUASISTATIC_REVERSE);
    assertEquals("sysid-quasistatic-reverse-Test", leg.getName());
    assertEquals(Set.of(motors.subsystem), leg.getRequirements());
  }

  @Test
  public void shouldRequireAtLeastOneTarget() {
    SysIdConfig<VoltageUnit> config = config().build();
    Target<VoltageUnit> target = new Target<>(core, VOLTS, Optional.empty());
    assertThrows(
        IllegalArgumentException.class,
        () -> new CustomSysIdRoutine<>(config, motors.subsystem, List.of()));
    assertThrows(
        NullPointerException.class,
        () -> new CustomSysIdRoutine<>(null, motors.subsystem, List.of(target)));
    assertThrows(
        NullPointerException.class, () -> new CustomSysIdRoutine<>(config, null, List.of(target)));
    assertThrows(
        NullPointerException.class, () -> new Target<VoltageUnit>(null, VOLTS, Optional.empty()));
    assertThrows(
        NullPointerException.class, () -> new Target<VoltageUnit>(core, null, Optional.empty()));
    assertThrows(NullPointerException.class, () -> new Target<>(core, VOLTS, null));
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
}
