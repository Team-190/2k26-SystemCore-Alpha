package edu.wpi.team190.gompeilib.io.motor.interfaces;

import static org.junit.jupiter.api.Assertions.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.control.Gains;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetryAutoLogged;
import edu.wpi.team190.gompeilib.io.motor.outputs.MotorRequest;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

public class MotorIOTest extends GompeiLibTestBase {

  @Test
  public void testDefaultMethods() throws IllegalAccessException {
    List<MotorRequest> sent = new ArrayList<>();
    MotorIO io = sent::add;
    MotorTelemetryAutoLogged telemetry = new MotorTelemetryAutoLogged();

    io.updateInputs(telemetry);
    io.setPosition(Rotations.of(2.0));
    io.updateGains(Gains.builder().withKP(1.0).build(), GainSlot.ONE);
    io.clearStickyFaults();
    io.setControl(new OpenLoopRequest.VoltageOutput(Volts.of(1.0)));

    assertFalse(io.isConnected());
    assertEquals(1, sent.size());
    IOTestSupport.assertZeroDefaults(telemetry);
  }

  @Test
  public void testReplayAppliesNothing() throws IllegalAccessException {
    MotorIO replay = MotorIO.replay();
    MotorTelemetryAutoLogged telemetry = new MotorTelemetryAutoLogged();

    assertDoesNotThrow(() -> replay.setControl(new OpenLoopRequest.VoltageOutput(Volts.of(12.0))));
    replay.updateInputs(telemetry);

    IOTestSupport.assertZeroDefaults(telemetry);
  }

  @Test
  public void testReplayFactory() {
    assertEquals("Replay", MotorIO.REPLAY_FACTORY.family());
    assertFalse(MotorIO.REPLAY_FACTORY.supportsHardwareFollowing());
    assertNotNull(MotorIO.REPLAY_FACTORY.create(null));
  }
}
