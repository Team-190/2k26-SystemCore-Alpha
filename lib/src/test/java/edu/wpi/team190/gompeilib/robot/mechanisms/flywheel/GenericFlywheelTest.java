package edu.wpi.team190.gompeilib.robot.mechanisms.flywheel;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.FollowerMotor;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.SlotConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.NeutralRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.RequestOptions;
import edu.wpi.team190.gompeilib.io.motor.outputs.VelocityRequest;
import edu.wpi.team190.gompeilib.io.physics.FlywheelMechanismSim;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismCore;
import edu.wpi.team190.gompeilib.sysid.CustomSysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.ArgumentCaptor;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.CurrentUnit;
import org.wpilib.units.VoltageUnit;

public class GenericFlywheelTest extends GompeiLibTestBase {
  private final List<MotorIO> motors = new ArrayList<>();
  private MotorIOFactory factory;
  private MotorConfig builtConfig;
  private Consumer<MotorTelemetry> reading = t -> {};
  private Subsystem subsystem;
  private GenericFlywheelConstants constants;
  private Gains voltageGains;
  private Gains torqueGains;

  @BeforeEach
  public void setUp() {
    motors.clear();
    factory = mock(MotorIOFactory.class);
    when(factory.family()).thenReturn("Mock");
    when(factory.supportsHardwareFollowing()).thenReturn(true);
    when(factory.create(any()))
        .thenAnswer(
            invocation -> {
              if (builtConfig == null) {
                builtConfig = invocation.getArgument(0);
              }
              MotorIO motor = mock(MotorIO.class);
              doAnswer(
                      update -> {
                        reading.accept(update.getArgument(0));
                        return null;
                      })
                  .when(motor)
                  .updateInputs(any());
              motors.add(motor);
              return motor;
            });
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("TestFlywheelSubsystem");

    voltageGains = Gains.builder().withPrefix("flywheel/voltage").build();
    torqueGains = Gains.builder().withPrefix("flywheel/torque").build();
    constants =
        GenericFlywheelConstants.builder()
            .withMotor(
                MotorConfig.builder()
                    .withCanId(1)
                    .withBus(CANBusId.named("rio"))
                    .withCurrentLimits(
                        CurrentLimits.fromDoubles()
                            .withSupplyCurrentLimit(40.0)
                            .withStatorCurrentLimit(60.0)
                            .build())
                    .withGearRatio(2.0)
                    .withSlot(GainSlot.ZERO, new SlotConfig(voltageGains))
                    .withSlot(GainSlot.ONE, new SlotConfig(torqueGains))
                    .withFollower(new FollowerMotor(2, MotorAlignment.OPPOSED))
                    .build())
            .withFlywheelGearbox(DCMotor.getKrakenX60Foc(2))
            .withMomentOfInertia(KilogramSquareMeters.of(0.01))
            .withConstraints(
                AngularVelocityConstraints.builder()
                    .withPrefix("flywheel/constraints")
                    .withMaxVelocity(RadiansPerSecond.of(500))
                    .withMaxAcceleration(RadiansPerSecondPerSecond.of(1000))
                    .withGoalTolerance(RadiansPerSecond.of(10))
                    .build())
            .withVelocityOffsetStep(RadiansPerSecond.of(5))
            .withVoltageOffsetStep(Volts.of(1))
            .build();
  }

  private static OpenLoopRequest.VoltageOutput voltage(double volts) {
    return new OpenLoopRequest.VoltageOutput(Volts.of(volts), true, RequestOptions.DEFAULT);
  }

  private static VelocityRequest.VelocityVoltage velocityVoltage(double radPerSec) {
    return new VelocityRequest.VelocityVoltage(
        RadiansPerSecond.of(radPerSec),
        RadiansPerSecondPerSecond.of(0.0),
        Volts.of(0.0),
        true,
        GainSlot.ZERO,
        RequestOptions.DEFAULT);
  }

  private static VelocityRequest.VelocityCurrent velocityCurrent(double radPerSec, double amps) {
    return new VelocityRequest.VelocityCurrent(
        RadiansPerSecond.of(radPerSec),
        RadiansPerSecondPerSecond.of(0.0),
        Amps.of(amps),
        GainSlot.ONE,
        RequestOptions.DEFAULT);
  }

  @Test
  public void testAddsTheProfileAndBuildsFollowers() {
    new GenericFlywheel(factory, subsystem, constants, "Test");

    assertEquals(
        MotionProfileConfig.trapezoidal(
            RadiansPerSecond.of(500), RadiansPerSecondPerSecond.of(1000)),
        builtConfig.motionProfile);
    assertEquals(2, motors.size());
  }

  @Test
  public void testPeriodicRunsTheCurrentState() {
    try (MockedStatic<Logger> logger = mockStatic(Logger.class)) {
      GenericFlywheel flywheel = new GenericFlywheel(factory, subsystem, constants, "Test");
      MotorIO leader = motors.get(0);
      assertEquals(MechanismCore.IDLE, flywheel.getMode());

      flywheel.periodic();
      verify(leader).updateInputs(any());
      logger.verify(() -> Logger.processInputs(eq("TestFlywheelSubsystem/Test/Leader"), any()));
      verify(leader, never()).setControl(any());
      verify(motors.get(1)).setControl(isA(FollowerRequest.Follow.class));

      flywheel.setVoltageGoal(Volts.of(6.0));
      flywheel.periodic();
      verify(leader).setControl(voltage(6.0));

      flywheel.setVoltageGoal(
          new Setpoint<VoltageUnit>(Volts.of(3.0), Volts.of(1), Volts.of(-12), Volts.of(12)));
      flywheel.periodic();
      verify(leader).setControl(voltage(3.0));

      flywheel.setVelocityGoal(RadiansPerSecond.of(100.0));
      assertEquals("VelocityVoltage", flywheel.getMode());
      flywheel.periodic();
      verify(leader).setControl(velocityVoltage(100.0));

      flywheel.setVelocityGoal(
          new Setpoint<AngularVelocityUnit>(
              RadiansPerSecond.of(80.0),
              RadiansPerSecond.of(5.0),
              RadiansPerSecond.of(-500.0),
              RadiansPerSecond.of(500.0)));
      flywheel.periodic();
      verify(leader).setControl(velocityVoltage(80.0));

      flywheel.setVelocityGoal(RadiansPerSecond.of(50.0), Amps.of(2.0));
      assertEquals("VelocityCurrent", flywheel.getMode());
      flywheel.periodic();
      verify(leader).setControl(velocityCurrent(50.0, 2.0));

      flywheel.setVelocityGoal(
          new Setpoint<AngularVelocityUnit>(
              RadiansPerSecond.of(30.0),
              RadiansPerSecond.of(5.0),
              RadiansPerSecond.of(-500.0),
              RadiansPerSecond.of(500.0)),
          Amps.of(3.0));
      flywheel.periodic();
      verify(leader).setControl(velocityCurrent(30.0, 3.0));

      assertEquals(Amps.of(3.0), flywheel.getVelocityFeedforward());

      flywheel.setCurrentGoal(Amps.of(4.0));
      assertEquals("Current", flywheel.getMode());
      flywheel.periodic();
      verify(leader).setControl(new OpenLoopRequest.CurrentOutput(Amps.of(4.0)));

      flywheel.stop();
      assertEquals(MechanismCore.STOP, flywheel.getMode());
      flywheel.periodic();
      verify(leader).setControl(new NeutralRequest.Neutral());
    }
  }

  @Test
  public void testGoalsCompareAgainstTheLeader() {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      GenericFlywheel flywheel =
          new GenericFlywheel(
              factory,
              subsystem,
              constants,
              "Test",
              new Setpoint<>(
                  RadiansPerSecond.of(100.0),
                  RadiansPerSecond.of(5.0),
                  RadiansPerSecond.of(-500.0),
                  RadiansPerSecond.of(500.0)));
      reading =
          t -> {
            t.velocity = RadiansPerSecond.of(95.0);
            t.position = Radians.of(12.0);
            t.appliedVoltage = Volts.of(0.3);
            t.torqueCurrent = Amps.of(0.2);
          };
      flywheel.periodic();

      assertEquals(95.0, flywheel.getVelocity().in(RadiansPerSecond), 1e-9);
      assertEquals(12.0, flywheel.getPosition().in(Radians), 1e-9);
      assertEquals(List.of(Amps.of(0.2), Amps.of(0.2)), flywheel.getTorqueCurrents());
      assertTrue(flywheel.atVelocityGoal());
      assertFalse(flywheel.atVelocityGoal(RadiansPerSecond.of(200.0)));
      assertTrue(flywheel.atVoltageGoal());
      assertFalse(flywheel.atVoltageGoal(Volts.of(3.0)));
      assertTrue(flywheel.atCurrentGoal());
      assertFalse(flywheel.atCurrentGoal(Amps.of(5.0)));

      // Waits a loop for periodic to apply the goal before checking it
      Command wait = flywheel.waitUntilAtVelocityGoal();
      wait.initialize();
      assertFalse(wait.isFinished());
    }
  }

  @Test
  public void testForwardsGainsAndConstraints() throws Exception {
    GenericFlywheel flywheel = new GenericFlywheel(factory, subsystem, constants, "Test");
    MotorIO leader = motors.get(0);

    Gains gains = Gains.builder().withKP(3.0).build();
    flywheel.updateGains(gains, GainSlot.ZERO);
    verify(leader).updateGains(gains, GainSlot.ZERO);

    flywheel.updateConstraints(
        AngularVelocityConstraints.builder()
            .withMaxVelocity(RadiansPerSecond.of(1.0))
            .withMaxAcceleration(RadiansPerSecondPerSecond.of(2.0))
            .build());
    ArgumentCaptor<MotionProfileConfig> profile =
        ArgumentCaptor.forClass(MotionProfileConfig.class);
    verify(leader).updateMotionProfile(profile.capture());
    assertEquals(2.0, profile.getValue().acceleration().in(RadiansPerSecondPerSecond), 1e-9);

    var lastNotified = Gains.class.getDeclaredField("lastNotifiedValues");
    lastNotified.setAccessible(true);
    lastNotified.set(torqueGains, null);
    var notify = Gains.class.getDeclaredMethod("notifyListeners");
    notify.setAccessible(true);
    notify.invoke(torqueGains);
    verify(leader).updateGains(torqueGains, GainSlot.ONE);
  }

  @Test
  public void testSimMechanism() {
    assertInstanceOf(FlywheelMechanismSim.class, GenericFlywheel.simMechanism(constants));
  }

  @Test
  @SuppressWarnings("unchecked") // Reading the generic routine fields back by reflection
  public void testSysIdRoutinesDriveTheMotors() throws Exception {
    try (MockedStatic<Logger> _ = mockStatic(Logger.class)) {
      GenericFlywheel flywheel = new GenericFlywheel(factory, subsystem, constants, "Test");
      MotorIO leader = motors.get(0);

      Command voltage = flywheel.sysIdRoutineVoltage();
      voltage.initialize();
      assertEquals(MechanismCore.IDLE, flywheel.getMode());
      flywheel.sysIdRoutineTorque().initialize();

      var voltageField = GenericFlywheel.class.getDeclaredField("voltageCharacterizationRoutine");
      voltageField.setAccessible(true);
      Command voltageQuasistatic =
          ((CustomSysIdRoutine<VoltageUnit>) voltageField.get(flywheel))
              .quasistatic(CustomSysIdRoutine.Direction.kForward);
      voltageQuasistatic.initialize();
      voltageQuasistatic.execute();
      voltageQuasistatic.end(true);
      verify(leader, atLeastOnce()).setControl(isA(OpenLoopRequest.VoltageOutput.class));

      var torqueField = GenericFlywheel.class.getDeclaredField("torqueCharacterizationRoutine");
      torqueField.setAccessible(true);
      Command torqueQuasistatic =
          ((CustomSysIdRoutine<CurrentUnit>) torqueField.get(flywheel))
              .quasistatic(CustomSysIdRoutine.Direction.kForward);
      torqueQuasistatic.initialize();
      torqueQuasistatic.execute();
      torqueQuasistatic.end(true);
      verify(leader, atLeastOnce()).setControl(isA(OpenLoopRequest.CurrentOutput.class));
    }
  }
}
