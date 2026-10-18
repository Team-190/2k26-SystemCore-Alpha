package edu.wpi.team190.gompeilib.io.physics;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.MomentOfInertia;

/** A single-jointed arm. The gearbox output is the arm joint. */
public class ArmMechanismSim extends AbstractMechanismSim {
  private final SingleJointedArmSim sim;

  /**
   * Creates the model.
   *
   * @param gearbox the motors driving the arm
   * @param gearRatio rotor rotations per arm rotation
   * @param momentOfInertia the arm's moment of inertia about the joint
   * @param armLength the arm's length
   * @param minAngle the lowest angle
   * @param maxAngle the highest angle
   * @param simulateGravity whether gravity pulls the arm down
   * @param startingAngle the starting angle
   */
  public ArmMechanismSim(
      DCMotor gearbox,
      double gearRatio,
      MomentOfInertia momentOfInertia,
      Distance armLength,
      Angle minAngle,
      Angle maxAngle,
      boolean simulateGravity,
      Angle startingAngle) {
    super(gearbox);
    sim =
        new SingleJointedArmSim(
            gearbox,
            gearRatio,
            momentOfInertia.in(KilogramSquareMeters),
            armLength.in(Meters),
            minAngle.in(Radians),
            maxAngle.in(Radians),
            simulateGravity,
            startingAngle.in(Radians));
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
    return Radians.of(sim.getAngle());
  }

  @Override
  public AngularVelocity velocity() {
    return RadiansPerSecond.of(sim.getVelocity());
  }

  @Override
  public Current currentDraw() {
    return Amps.of(sim.getCurrentDraw());
  }
}
