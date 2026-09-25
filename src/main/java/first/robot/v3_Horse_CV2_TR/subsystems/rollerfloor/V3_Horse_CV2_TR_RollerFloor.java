package first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor;

import static org.wpilib.units.Units.Volts;

import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRoller;
import edu.wpi.team190.gompeilib.subsystems.generic.roller.GenericRollerIO;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloorConstants.RollerFloorState;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Voltage;

public class V3_Horse_CV2_TR_RollerFloor extends SubsystemBase {
  private final GenericRoller rollerFloor;
  @Getter private RollerFloorState rollerFloorGoal;
  private final Setpoint<VoltageUnit> rollerFloorSetpoint, rollerFloorOverrideSetpoint;

  public V3_Horse_CV2_TR_RollerFloor(GenericRollerIO rollerFloorIO) {

    rollerFloorSetpoint =
        new Setpoint<>(
            V3_Horse_CV2_TR_RollerFloorConstants.ROLLER_FLOOR_RUN_VOLTAGE,
            V3_Horse_CV2_TR_RollerFloorConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));
    rollerFloorOverrideSetpoint =
        new Setpoint<>(
            V3_Horse_CV2_TR_RollerFloorConstants.ROLLER_FLOOR_OVERRIDE_VOLTAGE,
            V3_Horse_CV2_TR_RollerFloorConstants.ROLLER_FLOOR_CONSTANTS.voltageOffsetStep,
            Volts.of(-12),
            Volts.of(12));
    rollerFloor =
        new GenericRoller(
            rollerFloorIO,
            this,
            V3_Horse_CV2_TR_RollerFloorConstants.ROLLER_FLOOR_CONSTANTS,
            " Floor",
            rollerFloorSetpoint);
  }

  @Override
  public void periodic() {
    switch (rollerFloorGoal) {
      case RUN:
        runRollerFloor(rollerFloorSetpoint);
        break;
      case OVERRIDE:
        runRollerFloor(rollerFloorOverrideSetpoint);
        break;
      case STOP:
        stopRollerFloor();
        break;
      default:
        runRollerFloor(rollerFloorSetpoint);
        break;
    }
    Logger.recordOutput(
        "Elastic/Hopper/RollerFloor/Voltage Magnitude",
        String.format("%.1f", Math.abs(rollerFloor.getVoltageGoal().getSetpoint().in(Volts))));
    Logger.recordOutput(
        "Elastic/Hopper/RollerFloor/Voltage Offset",
        String.format("%.1f", rollerFloor.getVoltageGoal().getOffset().in(Volts)));
  }

  public Command setState(RollerFloorState state) {
    return Commands.runOnce(() -> rollerFloorGoal = state);
  }

  public void setRollerFloorVoltage(Voltage voltage) {
    rollerFloorSetpoint.setSetpoint(voltage);
  }

  public void setOverrideRollerFloorVoltage(Voltage voltage) {
    rollerFloorOverrideSetpoint.setSetpoint(voltage);
  }

  public void runRollerFloor(Setpoint<VoltageUnit> voltageSetpoint) {
    rollerFloor.setVoltageGoal(voltageSetpoint);
  }

  public void stopRollerFloor() {
    rollerFloor.setVoltageGoal(Volts.of(0));
  }

  public Command incrementRollerFloorVelocity() {
    return Commands.either(
        Commands.runOnce(rollerFloorSetpoint::increment),
        Commands.either(
            Commands.runOnce(rollerFloorOverrideSetpoint::increment),
            Commands.runOnce(() -> stopRollerFloor()),
            () -> rollerFloorGoal.equals(RollerFloorState.OVERRIDE)),
        () -> rollerFloorGoal.equals(RollerFloorState.RUN));
  }

  public Command decrementRollerFloorVelocity() {
    return Commands.either(
        Commands.runOnce(rollerFloorSetpoint::decrement),
        Commands.either(
            Commands.runOnce(rollerFloorOverrideSetpoint::decrement),
            Commands.runOnce(() -> stopRollerFloor()),
            () -> rollerFloorGoal.equals(RollerFloorState.OVERRIDE)),
        () -> rollerFloorGoal.equals(RollerFloorState.RUN));
  }
}
