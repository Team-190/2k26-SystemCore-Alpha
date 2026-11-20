package edu.wpi.team190.gompeilib.robot.mechanisms.extension;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import edu.wpi.team190.gompeilib.io.motor.outputs.OpenLoopRequest;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import edu.wpi.team190.gompeilib.robot.common.homing.CurrentHoming;
import java.util.function.Consumer;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.junction.Logger;
import org.mockito.MockedStatic;
import org.wpilib.command2.Command;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.DistanceUnit;

public class HomingExtensionTest extends GompeiLibTestBase {
  private static final CurrentHoming.Config<DistanceUnit> HOMING =
      new CurrentHoming.Config<>(
          CurrentHoming.Output.volts(Volts.of(-1.0)), Amps.of(-20.0), Amps.of(5.0), Meters.of(0.0));

  private MockedStatic<Logger> logger;
  private MotorIO motor;
  private MotorIOFactory factory;
  private Subsystem subsystem;
  private Consumer<MotorTelemetry> reading = t -> {};

  @BeforeEach
  public void setUp() {
    logger = mockStatic(Logger.class);
    motor = mock(MotorIO.class);
    doAnswer(
            invocation -> {
              reading.accept(invocation.getArgument(0));
              return null;
            })
        .when(motor)
        .updateInputs(any());
    factory = mock(MotorIOFactory.class);
    when(factory.family()).thenReturn("Mock");
    when(factory.create(any())).thenReturn(motor);
    subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    when(subsystem.getName()).thenReturn("Test");
  }

  @AfterEach
  public void tearDown() {
    logger.close();
  }

  private static ExtensionConstants constants() {
    return ExtensionConstants.builder()
        .withMotor(
            MotorConfig.builder()
                .withCanId(5)
                .withBus(CANBusId.named("rio"))
                .withCurrentLimits(new CurrentLimits(40, 40))
                .withGearRatio(10.0)
                .build())
        .withDistancePerRadian(Meters.of(0.02))
        .withExtensionParameters(
            ExtensionConstants.ExtensionParameters.builder()
                .withCarriageGearbox(DCMotor.getNeo550(1))
                .withMinLength(Meters.of(0.0))
                .withMaxLength(Meters.of(1.5))
                .withPlant(
                    PlantCharacterization.ofElevator(
                            DCMotor.getNeo550(1), 10.0, Kilograms.of(15.0), Meters.of(0.02))
                        .withKG(Volts.of(0))
                        .build())
                .build())
        .withConstraints(LinearConstraints.builder().build())
        .withVoltageOffsetStep(Volts.of(0.5))
        .withHeightOffsetStep(Meters.of(0.05))
        .withCharacterization(
            CharacterizationConfig.voltage()
                .withRampRate(VoltsPerSecond.ofNative(1))
                .withStepOutput(Volts.of(3))
                .withQuasistaticTimeout(Seconds.of(3))
                .withDynamicTimeout(Seconds.of(3))
                .build())
        .build();
  }

  @Test
  public void testHomesIntoTheStopAndResetsItsPosition() {
    HomingExtension extension =
        new HomingExtension(factory, subsystem, "Extension", constants(), HOMING);
    assertSame(HOMING, extension.homing().getConfig());

    Command home = extension.home();
    home.initialize();
    extension.periodic();
    home.execute();
    assertEquals("Homing", extension.getMode());
    verify(motor)
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(-1.0), true, CurrentHoming.OPTIONS));

    reading = t -> t.torqueCurrent = Amps.of(-20.0);
    extension.periodic();
    for (int i = 0; i < 5 && !home.isFinished(); i++) {
      home.execute();
    }
    assertTrue(home.isFinished());
    verify(motor).setPosition(Radians.of(0.0));
  }

  @Test
  public void testKeepsTheExtensionsGoals() {
    Setpoint<DistanceUnit> goal = new Setpoint<>(Meters.of(0.5), Meters.of(0.05));
    HomingExtension extension =
        new HomingExtension(factory, subsystem, "Extension", constants(), HOMING, goal);
    assertSame(goal, extension.getPositionGoal());
    assertEquals(0.0, extension.getVoltageGoal().getNewSetpoint().in(Volts), 1e-9);
  }

  @Test
  public void shouldRequireAHomingConfig() {
    assertThrows(
        NullPointerException.class,
        () -> new HomingExtension(factory, subsystem, "Extension", constants(), null));
  }
}
