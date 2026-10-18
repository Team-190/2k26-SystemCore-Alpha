package edu.wpi.team190.gompeilib.io.physics;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.MomentOfInertia;

/** A geared inertia with no external load, such as a roller or turret. */
public class DCMotorMechanismSim extends AbstractMechanismSim {
  private final DCMotorSim sim;

  /**
   * Creates the model.
   *
   * @param gearbox the motors driving the mechanism
   * @param momentOfInertia the mechanism's moment of inertia
   * @param gearRatio rotor rotations per output rotation
   */
  public DCMotorMechanismSim(DCMotor gearbox, MomentOfInertia momentOfInertia, double gearRatio) {
    super(gearbox);
    sim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
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
  }

  @Override
  public Angle position() {
    return Radians.of(sim.getAngularPosition());
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
