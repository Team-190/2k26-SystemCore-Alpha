package edu.wpi.team190.gompeilib.robot.mechanisms;

import static org.mockito.Mockito.*;
import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.control.Setpoint;
import edu.wpi.team190.gompeilib.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.io.base.CANBusId;
import edu.wpi.team190.gompeilib.io.motor.MotorControllerGroup;
import edu.wpi.team190.gompeilib.io.motor.config.CurrentLimits;
import edu.wpi.team190.gompeilib.io.motor.config.GainSlot;
import edu.wpi.team190.gompeilib.io.motor.config.MotionProfileConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.motor.inputs.MotorTelemetry;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIO;
import edu.wpi.team190.gompeilib.io.motor.interfaces.MotorIOFactory;
import java.util.Optional;
import java.util.function.Consumer;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;

/**
 * Shared fixtures for the mechanism-layer tests: a motor group over one mock motor whose telemetry
 * a test sets, a reflective trigger for tunable change listeners, and a test mechanism that
 * implements every capability.
 */
public final class MechanismTestSupport {
  /** A motor group over a single mock motor. */
  public static final class TestMotors {
    public final MotorIO motor = mock(MotorIO.class);
    public final MotorControllerGroup group;
    public final Subsystem subsystem = mock(Subsystem.class, CALLS_REAL_METHODS);
    private Consumer<MotorTelemetry> reading = t -> {};

    public TestMotors() {
      doAnswer(
              invocation -> {
                reading.accept(invocation.getArgument(0));
                return null;
              })
          .when(motor)
          .updateInputs(any());
      MotorIOFactory factory = mock(MotorIOFactory.class);
      when(factory.family()).thenReturn("Mock");
      when(factory.create(any())).thenReturn(motor);
      when(subsystem.getName()).thenReturn("Test");
      group = new MotorControllerGroup(config(), factory);
    }

    /**
     * Sets what the motor reports and reads it into the group's telemetry.
     *
     * @param reading writes the telemetry
     */
    public void read(Consumer<MotorTelemetry> reading) {
      this.reading = reading;
      group.periodic("Test/Mechanism");
    }

    public MechanismCore core() {
      return new MechanismCore(group, subsystem, "Mechanism");
    }
  }

  /** A mechanism with every capability, for testing the interfaces' defaults. */
  public static final class TestMechanism
      implements VoltageControllable,
          CurrentControllable,
          DutyCycleControllable,
          DifferentialControllable<AngleUnit, AngleUnit>,
          PositionControllable<AngleUnit>,
          VelocityControllable<AngularVelocityUnit>,
          MotionReadable<AngleUnit, AngularVelocityUnit>,
          ConstraintConfigurable<AngularPositionConstraints>,
          Homeable,
          Characterizable,
          EncoderSeedable {
    public final MechanismCore core;
    public final VoltageControl voltage;
    public final CurrentControl current;
    public final DutyCycleControl dutyCycle;
    public final PositionControl<AngleUnit> position;
    public final VelocityControl<AngularVelocityUnit> velocity;
    public final DifferentialControl<AngleUnit, AngleUnit> differential;
    public final CurrentHoming<AngleUnit> homing;
    public final EncoderSeed encoder;

    public TestMechanism(TestMotors motors) {
      core = motors.core();
      voltage = new VoltageControl(core, Volts.of(0.5), true);
      current = new CurrentControl(core, new Setpoint<>(Amps.of(0), Amps.of(1)));
      dutyCycle = new DutyCycleControl(core, 0.05, true);
      position =
          new PositionControl<>(
              core,
              CONVERSION,
              new Setpoint<>(Radians.of(0), Radians.of(0.1)),
              () -> Radians.of(0.05),
              true);
      velocity =
          new VelocityControl<>(
              core,
              CONVERSION,
              new Setpoint<>(RadiansPerSecond.of(0), RadiansPerSecond.of(1)),
              () -> RadiansPerSecond.of(1),
              true,
              GainSlot.ZERO,
              GainSlot.ONE);
      differential =
          new DifferentialControl<>(
              core,
              CONVERSION,
              CONVERSION,
              new Setpoint<>(Radians.of(0), Radians.of(0.1)),
              new Setpoint<>(Radians.of(0), Radians.of(0.1)),
              () -> Radians.of(0.05),
              () -> Radians.of(0.05),
              () -> CONVERSION.fromMotorPosition(core.getMotors().leader().position),
              () -> CONVERSION.fromMotorPosition(core.getMotors().leader().position),
              true);
      homing =
          new CurrentHoming<>(
              core,
              position,
              new CurrentHoming.Config<>(Volts.of(-1), Amps.of(-20), Amps.of(1), Radians.of(0)),
              true);
      encoder = new EncoderSeed(core, Optional.empty());
    }

    @Override
    public MechanismCore core() {
      return core;
    }

    @Override
    public void periodic() {
      core.periodic();
    }

    @Override
    public VoltageControl voltageControl() {
      return voltage;
    }

    @Override
    public CurrentControl currentControl() {
      return current;
    }

    @Override
    public DutyCycleControl dutyCycleControl() {
      return dutyCycle;
    }

    @Override
    public PositionControl<AngleUnit> positionControl() {
      return position;
    }

    @Override
    public VelocityControl<AngularVelocityUnit> velocityControl() {
      return velocity;
    }

    @Override
    public DifferentialControl<AngleUnit, AngleUnit> differentialControl() {
      return differential;
    }

    @Override
    public MechanismConversion<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit>
        conversion() {
      return CONVERSION;
    }

    @Override
    public MotionProfileConfig motionProfile(AngularPositionConstraints constraints) {
      return CONVERSION.trapezoidal(
          constraints.maxVelocity().get(), constraints.maxAcceleration().get());
    }

    @Override
    public CurrentHoming<?> homing() {
      return homing;
    }

    @Override
    public EncoderSeed encoderSeed() {
      return encoder;
    }
  }

  public static final MechanismConversion<AngleUnit, AngularVelocityUnit, AngularAccelerationUnit>
      CONVERSION = MechanismConversion.angular();

  private MechanismTestSupport() {}

  /**
   * Returns a minimal motor config.
   *
   * @return the config
   */
  public static MotorConfig config() {
    return MotorConfig.builder()
        .withCanId(1)
        .withBus(CANBusId.named("rio"))
        .withCurrentLimits(
            CurrentLimits.fromDoubles()
                .withSupplyCurrentLimit(40)
                .withStatorCurrentLimit(40)
                .build())
        .build();
  }

  /**
   * Fires a tunable's change listeners, as a dashboard edit would.
   *
   * @param tunable a {@code Gains} or constraints object
   */
  public static void fireChange(Object tunable) {
    try {
      var lastNotified = tunable.getClass().getDeclaredField("lastNotifiedValues");
      lastNotified.setAccessible(true);
      lastNotified.set(tunable, null);
      var notify = tunable.getClass().getDeclaredMethod("notifyListeners");
      notify.setAccessible(true);
      notify.invoke(tunable);
    } catch (ReflectiveOperationException e) {
      throw new IllegalStateException(e);
    }
  }
}
