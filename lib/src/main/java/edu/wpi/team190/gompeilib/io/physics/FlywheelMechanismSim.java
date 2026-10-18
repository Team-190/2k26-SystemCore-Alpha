package edu.wpi.team190.gompeilib.io.physics;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.MomentOfInertia;

/** A flywheel. The model tracks only velocity, so position is integrated from it. */
public class FlywheelMechanismSim extends AbstractMechanismSim {
  private final FlywheelSim sim;
  private double positionRadians = 0.0;

  /**
   * Creates the model.
   *
   * @param gearbox the motors driving the flywheel
   * @param momentOfInertia the flywheel's moment of inertia
   * @param gearRatio rotor rotations per flywheel rotation
   */
  public FlywheelMechanismSim(DCMotor gearbox, MomentOfInertia momentOfInertia, double gearRatio) {
    super(gearbox);
    sim =
        new FlywheelSim(
            Models.flywheelFromPhysicalConstants(
                gearbox, momentOfInertia.in(KilogramSquareMeters), gearRatio),
            gearbox);
  }

  @Override
  protected void applyVoltage(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  protected void step(double seconds) {
    sim.update(seconds);
    positionRadians += sim.getAngularVelocity() * seconds;
  }

  @Override
  public Angle position() {
    return Radians.of(positionRadians);
  }

  @Override
  public AngularVelocity velocity() {
    return RadiansPerSecond.of(sim.getAngularVelocity());
  }

  @Override
  public Current currentDraw() {
    return Amps.of(sim.getCurrentDraw());
  }
}
