package edu.wpi.team190.gompeilib.io.physics;

import static org.wpilib.units.Units.*;

import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;

/**
 * An elevator carriage on a drum. Height is reported as the drum's angle at the gearbox output, so
 * it matches what the motor measures; the subsystem converts back through the drum radius.
 */
public class ElevatorMechanismSim extends AbstractMechanismSim {
  private final ElevatorSim sim;
  private final double drumRadiusMeters;

  /**
   * Creates the model.
   *
   * @param gearbox the motors driving the elevator
   * @param gearRatio rotor rotations per drum rotation
   * @param carriageMass the carriage's mass
   * @param drumRadius the drum's radius
   * @param minHeight the lowest height
   * @param maxHeight the highest height
   * @param simulateGravity whether gravity pulls the carriage down
   * @param startingHeight the starting height
   */
  public ElevatorMechanismSim(
      DCMotor gearbox,
      double gearRatio,
      Mass carriageMass,
      Distance drumRadius,
      Distance minHeight,
      Distance maxHeight,
      boolean simulateGravity,
      Distance startingHeight) {
    super(gearbox);
    drumRadiusMeters = drumRadius.in(Meters);
    sim =
        new ElevatorSim(
            gearbox,
            gearRatio,
            carriageMass.in(Kilograms),
            drumRadiusMeters,
            minHeight.in(Meters),
            maxHeight.in(Meters),
            simulateGravity,
            startingHeight.in(Meters));
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
    return Radians.of(sim.getPosition() / drumRadiusMeters);
  }

  @Override
  public AngularVelocity velocity() {
    return RadiansPerSecond.of(sim.getVelocity() / drumRadiusMeters);
  }

  @Override
  public Current currentDraw() {
    return Amps.of(sim.getCurrentDraw());
  }
}
