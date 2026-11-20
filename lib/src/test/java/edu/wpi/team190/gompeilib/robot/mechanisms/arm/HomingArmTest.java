package edu.wpi.team190.gompeilib.robot.mechanisms.arm;

import static edu.wpi.team190.gompeilib.math.CustomUnits.*;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.GompeiLibTestBase;
import edu.wpi.team190.gompeilib.characterization.CharacterizationConfig;
import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
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
import org.wpilib.units.AngleUnit;

public class HomingArmTest extends GompeiLibTestBase {
  private static final CurrentHoming.Config<AngleUnit> HOMING =
      new CurrentHoming.Config<>(
          CurrentHoming.Output.volts(Volts.of(-1)), Amps.of(-20), Amps.of(1), Degrees.of(-90));

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

  static ArmConstants constants() {
    return ArmConstants.builder()
        .withMotor(
            MotorConfig.builder()
                .withCanId(10)
                .withBus(CANBusId.named("rio"))
                .withCurrentLimits(new CurrentLimits(40, 40))
                .withGearRatio(50.0)
                .build())
        .withArmParameters(
            ArmConstants.ArmParameters.builder()
                .withArmGearbox(DCMotor.getKrakenX60(1))
                .withMinAngle(Degrees.of(-90))
                .withMaxAngle(Degrees.of(90))
                .withPlant(
                    PlantCharacterization.ofInertia(
                            DCMotor.getKrakenX60(1), 50.0, KilogramSquareMeters.of(0.01))
                        .build())
                .build())
        .withConstraints(AngularPositionConstraints.builder().build())
        .withVoltageOffsetStep(Volts.of(0.5))
        .withPositionOffsetStep(Degrees.of(5))
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
    HomingArm arm = new HomingArm(factory, subsystem, "Arm", constants(), HOMING);
    assertSame(HOMING, arm.homing().getConfig());
    assertTrue(arm.core().components().contains(arm.homing()));

    Command home = arm.home();
    home.initialize();
    arm.periodic();
    home.execute();
    assertEquals("Homing", arm.getMode());
    verify(motor)
        .setControl(new OpenLoopRequest.VoltageOutput(Volts.of(-1), true, CurrentHoming.OPTIONS));

    reading = t -> t.torqueCurrent = Amps.of(-20);
    arm.periodic();
    for (int i = 0; i < 5 && !home.isFinished(); i++) {
      home.execute();
    }
    assertTrue(home.isFinished());
    verify(motor).setPosition(Degrees.of(-90));
  }

  @Test
  public void testKeepsTheArmsGoals() {
    Setpoint<AngleUnit> goal = new Setpoint<>(Degrees.of(10), Degrees.of(1));
    HomingArm arm = new HomingArm(factory, subsystem, "Arm", constants(), HOMING, goal);
    assertSame(goal, arm.getPositionGoal());
    assertEquals(0.0, arm.getVoltageGoal().getNewSetpoint().in(Volts), 1e-9);

    HomingArm defaults = new HomingArm(factory, subsystem, "Arm", constants(), HOMING);
    assertEquals(0.0, defaults.getPositionGoal().getNewSetpoint().in(Degrees), 1e-9);
  }

  @Test
  public void shouldRequireAHomingConfig() {
    assertThrows(
        NullPointerException.class,
        () -> new HomingArm(factory, subsystem, "Arm", constants(), null));
  }
}
