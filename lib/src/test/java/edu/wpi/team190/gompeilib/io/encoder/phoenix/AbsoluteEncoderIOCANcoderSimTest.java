package edu.wpi.team190.gompeilib.io.encoder.phoenix;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.ArgumentMatchers.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.encoder.config.SensorDirection;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.physics.MechanismSim;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class AbsoluteEncoderIOCANcoderSimTest extends GompeiLibTestBase {
  private final CANcoderConfigurator configurator = mock(CANcoderConfigurator.class);
  private final Map<CANcoder, CANcoderSimState> simStates = new HashMap<>();

  private static AbsoluteEncoderConfig config(SensorDirection direction) {
    return AbsoluteEncoderConfig.builder()
        .withCanId(3)
        .withBus(CANBusId.named("rio"))
        .withDirection(direction)
        .build();
  }

  private void withCANcoder(IOTestSupport.PhoenixBody<CANcoder> body) throws Exception {
    when(configurator.apply(any(CANcoderConfiguration.class), anyDouble()))
        .thenReturn(StatusCode.OK);
    IOTestSupport.withMockedPhoenix(
        CANcoder.class,
        (mock, context) -> {
          when(mock.getConfigurator()).thenReturn(configurator);
          CANcoderSimState simState = mock(CANcoderSimState.class);
          simStates.put(mock, simState);
          when(mock.getSimState()).thenReturn(simState);
        },
        body);
  }

  @Test
  public void testTurnsTheMagnetFromTheMechanism() throws Exception {
    MechanismSim mechanism = mock(MechanismSim.class);
    when(mechanism.position()).thenReturn(Rotations.of(0.5));
    when(mechanism.velocity()).thenReturn(RotationsPerSecond.of(2.0));
    withCANcoder(
        mocks -> {
          AbsoluteEncoderIOCANcoderSim io =
              new AbsoluteEncoderIOCANcoderSim(
                  config(SensorDirection.COUNTER_CLOCKWISE_POSITIVE), mechanism, 2.0);
          CANcoderSimState simState = simStates.get(mocks.device(0));

          io.updateInputs(new AbsoluteEncoderTelemetryAutoLogged());

          verify(simState).setSupplyVoltage(anyDouble());
          verify(simState).setRawPosition(Rotations.of(1.0));
          verify(simState).setVelocity(RotationsPerSecond.of(4.0));
          assertEquals(ChassisReference.CounterClockwise_Positive, simState.Orientation);
        });
  }

  @Test
  public void testOrientsAClockwiseEncoder() throws Exception {
    withCANcoder(
        mocks -> {
          new AbsoluteEncoderIOCANcoderSim(
              config(SensorDirection.CLOCKWISE_POSITIVE), mock(MechanismSim.class), 1.0);

          assertEquals(
              ChassisReference.Clockwise_Positive, simStates.get(mocks.device(0)).Orientation);
        });
  }
}
