package edu.wpi.team190.gompeilib.subsystems.elevator;

import static org.wpilib.units.Units.*;
import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class ElevatorIOTalonFXSim extends ElevatorIOTalonFX {
  private final ElevatorSim elevatorSim;

  private final TalonFXSimState elevatorController;

  public ElevatorIOTalonFXSim(ElevatorConstants constants) {
    super(constants);
    elevatorSim =
        new ElevatorSim(
            Models.elevatorFromPhysicalConstants(
                constants.elevatorParameters.ELEVATOR_MOTOR_CONFIG(),
                constants.elevatorParameters.CARRIAGE_MASS_KG(),
                constants.drumRadius,
                constants.elevatorGearRatio),
            constants.elevatorParameters.ELEVATOR_MOTOR_CONFIG(),
            constants.elevatorParameters.MIN_HEIGHT().in(Meters),
            constants.elevatorParameters.MAX_HEIGHT().in(Meters),
            true,
            constants.elevatorParameters.MIN_HEIGHT().in(Meters));

    elevatorController = super.talonFX.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double elevatorVoltage = elevatorController.getMotorVoltage();

    elevatorSim.setInputVoltage(elevatorVoltage);

    elevatorSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(
            elevatorSim.getPosition() * constants.elevatorGearRatio * constants.drumRadius,
            Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            elevatorSim.getVelocity() * constants.elevatorGearRatio * constants.drumRadius,
            RadiansPerSecond);
    elevatorController.setRawRotorPosition(rotorPosition);
    elevatorController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
