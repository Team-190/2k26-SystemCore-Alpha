package edu.wpi.team190.gompeilib.subsystems.generic.hood;

import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class GenericHoodIOTalonFXSim extends GenericHoodIOTalonFX {
  private final SingleJointedArmSim hoodSim;

  private final TalonFXSimState hoodController;

  public GenericHoodIOTalonFXSim(GenericHoodConstants constants) {
    super(constants);

    hoodSim =
        new SingleJointedArmSim(
            constants.motorConfig,
            constants.gearRatio,
            constants.momentOfInertia,
            constants.lengthMeters,
            constants.minAngle.getRadians(),
            constants.maxAngle.getRadians(),
            true,
            constants.minAngle.getRadians());

    hoodController = super.hoodMotor.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(GenericHoodIOInputs inputs) {
    hoodController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double hoodVoltage = hoodController.getMotorVoltage();

    hoodSim.setInputVoltage(hoodVoltage);

    hoodSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition = Angle.ofBaseUnits(hoodSim.getAngle() * constants.gearRatio, Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(hoodSim.getVelocity() * constants.gearRatio, RadiansPerSecond);
    hoodController.setRawRotorPosition(rotorPosition);
    hoodController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
