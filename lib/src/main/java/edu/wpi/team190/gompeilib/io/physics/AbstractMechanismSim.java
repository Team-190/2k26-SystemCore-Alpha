package edu.wpi.team190.gompeilib.io.physics;

import static org.wpilib.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;

/**
 * Shared behavior for the WPILib physics wrappers: driver registration, input voltage, and an
 * acceleration estimate for models that don't report one.
 */
public abstract class AbstractMechanismSim implements MechanismSim {
  private final DCMotor gearbox;
  private final List<Object> motors = new ArrayList<>();
  private Voltage inputVoltage = Volts.of(0.0);
  private AngularAcceleration acceleration = RadiansPerSecondPerSecond.of(0.0);

  protected AbstractMechanismSim(DCMotor gearbox) {
    this.gearbox = gearbox;
  }

  @Override
  public final boolean claimDriver(Object motor) {
    if (motors.stream().noneMatch(m -> m == motor)) {
      motors.add(motor);
    }
    return motors.get(0) == motor;
  }

  @Override
  public final int motorCount() {
    return Math.max(1, motors.size());
  }

  @Override
  public final Object driver() {
    return motors.isEmpty() ? null : motors.get(0);
  }

  @Override
  public final void setInputVoltage(Voltage voltage) {
    inputVoltage = voltage;
    applyVoltage(voltage.in(Volts));
  }

  @Override
  public final Voltage inputVoltage() {
    return inputVoltage;
  }

  @Override
  public final void update(Time dt) {
    double before = velocity().in(RadiansPerSecond);
    step(dt.in(Seconds));
    acceleration =
        RadiansPerSecondPerSecond.of((velocity().in(RadiansPerSecond) - before) / dt.in(Seconds));
  }

  @Override
  public AngularAcceleration acceleration() {
    return acceleration;
  }

  @Override
  public final DCMotor gearbox() {
    return gearbox;
  }

  /**
   * Applies a voltage to the underlying model.
   *
   * @param volts the voltage
   */
  protected abstract void applyVoltage(double volts);

  /**
   * Steps the underlying model.
   *
   * @param seconds the time step
   */
  protected abstract void step(double seconds);
}
