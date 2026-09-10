package edu.wpi.team190.gompeilib.subsystems.extension;

import static org.wpilib.units.Units.*;
import static org.wpilib.units.Units.Meters;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.ExtensionSim;
import org.wpilib.system.RobotController;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class ExtensionIOTalonFXSim extends ExtensionIOTalonFX {
  private final ExtensionSim extensionSim;

  private final TalonFXSimState extensionController;

  public ExtensionIOTalonFXSim(ExtensionConstants constants) {
    super(constants);
    extensionSim =
        new ExtensionSim(
            Models.extensionFromPhysicalConstants(
                constants.extensionParameters.ELEVATOR_MOTOR_CONFIG(),
                constants.extensionParameters.CARRIAGE_MASS_KG(),
                constants.drumRadius,
                constants.extensionGearRatio),
            constants.extensionParameters.ELEVATOR_MOTOR_CONFIG(),
            constants.extensionParameters.MIN_HEIGHT().in(Meters),
            constants.extensionParameters.MAX_HEIGHT().in(Meters),
            true,
            constants.extensionParameters.MIN_HEIGHT().in(Meters));

    extensionController = super.talonFX.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(ExtensionIOInputs inputs) {
    extensionController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double extensionVoltage = extensionController.getMotorVoltage();

    extensionSim.setInputVoltage(extensionVoltage);

    extensionSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(
            extensionSim.getPosition() * constants.extensionGearRatio * constants.drumRadius,
            Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            extensionSim.getVelocity() * constants.extensionGearRatio * constants.drumRadius,
            RadiansPerSecond);
    extensionController.setRawRotorPosition(rotorPosition);
    extensionController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
