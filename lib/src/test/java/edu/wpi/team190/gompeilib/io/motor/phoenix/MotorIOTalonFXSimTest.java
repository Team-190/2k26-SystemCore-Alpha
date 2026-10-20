package edu.wpi.team190.gompeilib.io.motor.phoenix;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorAlignment;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorDirection;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.FollowerRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.phoenix.PhoenixUtil;
import edu.wpi.team190.gompeilib.io.physics.DCMotorMechanismSim;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.MockedConstruction;
import org.mockito.MockedStatic;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;

public class MotorIOTalonFXSimTest extends GompeiLibTestBase {
  private static final double GEAR_RATIO = 4.0;

  private MechanismSim mechanism;
  private final Map<TalonFX, TalonFXSimState> simStates = new HashMap<>();

  @BeforeEach
  public void setUp() {
    mechanism =
        new DCMotorMechanismSim(
            DCMotor.getKrakenX60(1), KilogramSquareMeters.of(0.001), GEAR_RATIO);
  }

  private static MotorConfig config(int canId, MotorDirection direction) {
    return MotorConfig.builder()
        .withCanId(canId)
        .withBus(CANBusId.named("rio"))
        .withDirection(direction)
        .withGearRatio(GEAR_RATIO)
        .withCurrentLimits(
            CurrentLimits.fromDoubles()
                .withSupplyCurrentLimit(40.0)
                .withStatorCurrentLimit(80.0)
                .build())
        .build();
  }

  private interface Body {
    void run(MockedConstruction<TalonFX> talons) throws Exception;
  }

  private void withMockedTalonFX(Body body) throws Exception {
    TalonFXConfigurator configurator = mock(TalonFXConfigurator.class);
    when(configurator.apply(any(TalonFXConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);
    try (MockedConstruction<TalonFX> talons =
            mockConstruction(
                TalonFX.class,
                withSettings().defaultAnswer(IOTestSupport.phoenixSignalAnswer()),
                (mock, context) -> {
                  TalonFXSimState simState = mock(TalonFXSimState.class);
                  when(simState.getMotorVoltage()).thenReturn(6.0);
                  simStates.put(mock, simState);
                  when(mock.getConfigurator()).thenReturn(configurator);
                  when(mock.getSimState()).thenReturn(simState);
                });
        MockedStatic<BaseStatusSignal> _ = mockStatic(BaseStatusSignal.class);
        MockedStatic<PhoenixUtil> phoenix = mockStatic(PhoenixUtil.class, CALLS_REAL_METHODS)) {
      phoenix
          .when(() -> PhoenixUtil.registerSignals(any(), any(BaseStatusSignal[].class)))
          .thenAnswer(invocation -> null);
      body.run(talons);
    }
  }

  private TalonFXSimState simState(MockedConstruction<TalonFX> talons, int index) {
    return simStates.get(talons.constructed().get(index));
  }

  /** A motor the test steps itself, kept off the shared sim thread. */
  private MotorIOTalonFXSim motor(MotorConfig config) {
    return new MotorIOTalonFXSim(config, mechanism, null, false);
  }

  @AfterEach
  public void tearDown() {
    MotorIOTalonFXSim.stopSimThread();
  }

  @Test
  public void testSimThreadStepsTheMechanismOnItsOwn() throws Exception {
    withMockedTalonFX(
        talons -> {
          new MotorIOTalonFXSim(config(1, MotorDirection.COUNTER_CLOCKWISE_POSITIVE), mechanism);

          long deadline = System.nanoTime() + 2_000_000_000L;
          while (mechanism.velocity().in(RadiansPerSecond) <= 0 && System.nanoTime() < deadline) {
            Thread.sleep(5);
          }

          assertEquals(6.0, mechanism.inputVoltage().in(Volts), 1e-9);
          assertTrue(mechanism.velocity().in(RadiansPerSecond) > 0);
        });
  }

  @Test
  public void testDriverAppliesPhoenixVoltageToTheMechanism() throws Exception {
    withMockedTalonFX(
        talons -> {
          MotorIOTalonFXSim motor = motor(config(1, MotorDirection.COUNTER_CLOCKWISE_POSITIVE));

          motor.simulate(0.02);

          assertTrue(motor.isDriver());
          assertEquals(6.0, mechanism.inputVoltage().in(Volts), 1e-9);
          assertTrue(mechanism.velocity().in(RadiansPerSecond) > 0);
          TalonFXSimState sim = simState(talons, 0);
          verify(sim).setSupplyVoltage(anyDouble());
          verify(sim).setRawRotorPosition(mechanism.position().times(GEAR_RATIO));
          verify(sim).setRotorVelocity(mechanism.velocity().times(GEAR_RATIO));
          verify(sim).setRotorAcceleration(mechanism.acceleration().times(GEAR_RATIO));
        });
  }

  @Test
  public void testPassengerReadsButDoesNotDrive() throws Exception {
    withMockedTalonFX(
        talons -> {
          motor(config(1, MotorDirection.COUNTER_CLOCKWISE_POSITIVE));
          MotorIOTalonFXSim passenger = motor(config(2, MotorDirection.COUNTER_CLOCKWISE_POSITIVE));

          passenger.simulate(0.02);

          assertFalse(passenger.isDriver());
          assertEquals(0.0, mechanism.inputVoltage().in(Volts));
          verify(simState(talons, 1)).setRawRotorPosition(any(Angle.class));
          verify(simState(talons, 1)).setRotorVelocity(any(AngularVelocity.class));
          verify(simState(talons, 1)).setRotorAcceleration(any(AngularAcceleration.class));
          assertNull(simState(talons, 1).Orientation);
        });
  }

  @Test
  public void testMirroredOpposedPassengerIsOrientedClockwise() throws Exception {
    withMockedTalonFX(
        talons -> {
          motor(config(1, MotorDirection.COUNTER_CLOCKWISE_POSITIVE));
          motor(config(2, MotorDirection.CLOCKWISE_POSITIVE));

          assertEquals(ChassisReference.Clockwise_Positive, simState(talons, 1).Orientation);
        });
  }

  @Test
  public void testPassengerOfAnotherDriverKeepsItsOrientation() throws Exception {
    mechanism.claimDriver(new Object());
    withMockedTalonFX(
        talons -> {
          MotorIOTalonFXSim passenger = motor(config(2, MotorDirection.CLOCKWISE_POSITIVE));

          assertFalse(passenger.isDriver());
          assertNull(simState(talons, 0).Orientation);
        });
  }

  @Test
  public void testFollowRequestSetsOrientation() throws Exception {
    withMockedTalonFX(
        talons -> {
          motor(config(1, MotorDirection.COUNTER_CLOCKWISE_POSITIVE));
          MotorIOTalonFXSim follower = motor(config(2, MotorDirection.COUNTER_CLOCKWISE_POSITIVE));
          TalonFXSimState sim = simState(talons, 1);

          follower.setControl(new FollowerRequest.Follow(1, MotorAlignment.OPPOSED));
          assertEquals(ChassisReference.Clockwise_Positive, sim.Orientation);

          follower.setControl(new FollowerRequest.Follow(1, MotorAlignment.ALIGNED));
          assertEquals(ChassisReference.CounterClockwise_Positive, sim.Orientation);

          follower.setControl(new OpenLoopRequest.VoltageOutput(Volts.of(1.0)));
          assertEquals(ChassisReference.CounterClockwise_Positive, sim.Orientation);
          verify(talons.constructed().get(1), times(3)).setControl(any(ControlRequest.class));
        });
  }

  @Test
  public void testFactory() throws Exception {
    withMockedTalonFX(
        talons -> {
          MotorIOFactory factory = MotorIOTalonFXSim.factory(mechanism);

          assertEquals(MotorIOTalonFX.FAMILY, factory.family());
          assertTrue(factory.supportsHardwareFollowing());
          assertInstanceOf(
              MotorIOTalonFXSim.class,
              factory.create(config(1, MotorDirection.COUNTER_CLOCKWISE_POSITIVE)));
        });
  }
}
