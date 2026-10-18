package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.io.encoder.inputs.AbsoluteEncoderTelemetry;
import edu.wpi.team190.gompeilib.io.encoder.interfaces.AbsoluteEncoderIO;
import edu.wpi.team190.gompeilib.robot.mechanisms.MechanismTestSupport.TestMotors;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;

public class EncoderSeedTest extends GompeiLibTestBase {
  private MockedStatic<Logger> logger;
  private TestMotors motors;
  private MechanismCore core;

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    motors = new TestMotors();
    core = motors.core();
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  @Test
  public void testReadsLogsAndSeedsFromTheEncoder() {
    AbsoluteEncoderIO encoder = mock(AbsoluteEncoderIO.class);
    doAnswer(
            invocation -> {
              ((AbsoluteEncoderTelemetry) invocation.getArgument(0)).absolutePosition =
                  Rotations.of(0.125);
              return null;
            })
        .when(encoder)
        .updateInputs(any());
    EncoderSeed seed = new EncoderSeed(core, Optional.of(encoder));
    assertTrue(seed.isPresent());
    assertTrue(core.components().contains(seed));

    core.periodic();
    logger.verify(() -> Logger.processInputs(eq("Test/Mechanism/Encoder"), any()));

    seed.seedPosition();
    verify(motors.motor).setPosition(Rotations.of(0.125));
  }

  @Test
  public void testWithoutAnEncoderDoesNothing() {
    EncoderSeed seed = new EncoderSeed(core, Optional.empty());
    assertFalse(seed.isPresent());

    seed.periodic("T");
    seed.seedPosition();
    logger.verify(() -> Logger.processInputs(eq("T/Encoder"), any()), never());
    verify(motors.motor, never()).setPosition(any());
  }

  @Test
  public void shouldRequireAnOptional() {
    assertThrows(NullPointerException.class, () -> new EncoderSeed(core, null));
  }
}
