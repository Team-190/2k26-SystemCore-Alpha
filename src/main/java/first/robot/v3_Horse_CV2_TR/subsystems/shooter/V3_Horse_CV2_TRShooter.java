package first.robot.v3_Horse_CV2_TR.subsystems.shooter;

import static org.wpilib.units.Units.*;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHood;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHoodIO;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants.ShooterGoal;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

public class V3_Horse_CV2_TRShooter extends SubsystemBase {

  private final GenericFlywheel flywheel;
  private ShooterGoal shooterGoal;

  private Voltage overrideHoodVoltage;
  private Voltage overrideFlywheelVoltage;

  private final GenericHood hood;

  private final Trigger flywheelShootingTrigger;
  private final Trigger flywheelFeedingTrigger;

  private final Trigger hoodFeedingTrigger;

  public V3_Horse_CV2_TRShooter(GenericFlywheelIO flywheelIO, GenericHoodIO hoodIO) {

    Setpoint<AngularVelocityUnit> flywheelVelocitySetpoint =
        new Setpoint<AngularVelocityUnit>(
            RadiansPerSecond.of(0.0),
            V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS.velocityOffsetStep);
    Setpoint<VoltageUnit> flywheelVoltageSetpoint =
        new Setpoint<VoltageUnit>(
            Volts.of(0.0), V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS.voltageOffsetStep);

    Setpoint<AngleUnit> hoodAngleSetpoint =
        new Setpoint<AngleUnit>(
            Radians.of(0.0), V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS.offsetStep);
    Setpoint<VoltageUnit> hoodVoltageSetpoint =
        new Setpoint<VoltageUnit>(
            Volts.of(0.0), V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS.voltageStep);

    this.flywheel =
        new GenericFlywheel(
            flywheelIO,
            this,
            V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS,
            "",
            flywheelVelocitySetpoint,
            flywheelVoltageSetpoint);
    this.hood =
        new GenericHood(
            hoodIO,
            this,
            V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS,
            "",
            hoodAngleSetpoint,
            hoodVoltageSetpoint);
    this.shooterGoal = ShooterGoal.STOW;
    this.overrideHoodVoltage = Volts.of(0.0);
    this.overrideFlywheelVoltage = Volts.of(0.0);

    flywheelShootingTrigger =
        new Trigger(
                () ->
                    Math.abs(
                            flywheel
                                .getFlywheelVelocity()
                                .minus(flywheel.getVelocityGoal().getNewSetpoint())
                                .in(RadiansPerSecond))
                        <= V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS
                            .constraints
                            .goalTolerance()
                            .get(RadiansPerSecond))
            .debounce(.75, Debouncer.DebounceType.kFalling);
    flywheelFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        flywheel
                            .getFlywheelVelocity()
                            .minus(flywheel.getVelocityGoal().getNewSetpoint())
                            .in(RadiansPerSecond))
                    <= (V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS
                            .constraints
                            .goalTolerance()
                            .get(RadiansPerSecond)
                        + 35));

    hoodFeedingTrigger =
        new Trigger(
            () ->
                Math.abs(
                        hood.getAngle()
                            .getMeasure()
                            .minus(hood.getPositionGoal().getNewSetpoint())
                            .in(Radians))
                    <= (V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS
                            .constraints
                            .goalTolerance()
                            .get(Radians)
                        + Units.degreesToRadians(5)));
  }

  @Trace
  public void periodic() {

    switch (shooterGoal) {
      case STOW:
        hood.setPositionGoal(V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS.minAngle);
        flywheel.stop();
        break;
      case SCORE:
        hood.setPositionGoal(V3_Horse_CV2_TRRobotState.getScoreAngle());
        flywheel.setVelocityGoal(V3_Horse_CV2_TRRobotState.getScoreVelocity());
        break;
      case FEED:
        hood.setPositionGoal(V3_Horse_CV2_TRRobotState.getFeedAngle());
        flywheel.setVelocityGoal(V3_Horse_CV2_TRRobotState.getFeedVelocity());
        break;
      case OVERRIDE_HOOD:
        hood.setVoltageGoal(overrideHoodVoltage);
        break;
      case OVERRIDE_FLYWHEEL:
        flywheel.setVoltageGoal(overrideFlywheelVoltage);
        break;
      case ZERO:
        hood.setPositionGoal(Rotation2d.kZero);
        flywheel.stop();
        break;
      case STOP:
        hood.setVoltageGoal(Volts.of(0.0));
        flywheel.stop();
        break;
      default:
        break;
    }

    hood.periodic();
    flywheel.periodic();

    Logger.recordOutput("Shooter/Goal", shooterGoal);

    Logger.recordOutput(
        "Shooter/Hood/Goal Degrees",
        String.format("%.1f", hood.getPositionGoal().getSetpoint().in(Degrees)));
    Logger.recordOutput(
        "Shooter/Hood/Offset Degrees",
        String.format("%.1f", hood.getPositionGoal().getOffset().in(Degrees)));
    Logger.recordOutput(
        "Shooter/Flywheel/Velocity Offset",
        flywheel.getVelocityGoal().getOffset().in(RadiansPerSecond));
    Logger.recordOutput(
        "Shooter/Flywheel/Velocity Magnitude",
        (int) Math.abs(flywheel.getVelocityGoal().getSetpoint().in(RadiansPerSecond)));

    Logger.recordOutput(
        "Shooter/Flywheel Ready",
        V3_Horse_CV2_TRRobotState.isInAllianceZone()
            ? flywheelShootingTrigger.getAsBoolean()
            : flywheelFeedingTrigger.getAsBoolean());

    Logger.recordOutput(
        "Shooter/Hood At Goal",
        (V3_Horse_CV2_TRRobotState.isInAllianceZone())
            ? hood.atPositionGoal()
            : hoodFeedingTrigger.getAsBoolean());
  }

  public Command setGoal(ShooterGoal shooterGoal) {
    return this.runOnce(() -> this.shooterGoal = shooterGoal);
  }

  public boolean atGoal() {
    return hood.atPositionGoal() && flywheel.atVelocityGoal();
  }

  public Command waitUntilAtGoal() {
    return hood.waitUntilAtGoal().alongWith(flywheel.waitUntilAtGoal());
  }

  public Command waitUntilHoodAtGoal() {
    return hood.waitUntilAtGoal();
  }

  public Command waitUntilFlywheelAtGoal() {
    return flywheel.waitUntilAtGoal();
  }

  public Command hoodSysId() {
    return hood.runSysIdRoutine();
  }

  public Command flywheelSysId() {
    return flywheel.sysIdRoutineTorque();
  }

  public Command incrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::increment);
  }

  public Command decrementFlywheelVelocity() {
    return Commands.runOnce(flywheel.getVelocityGoal()::decrement);
  }

  public Command incrementHoodAngle() {
    return Commands.runOnce(hood.getPositionGoal()::increment);
  }

  public Command decrementHoodAngle() {
    return Commands.runOnce(hood.getPositionGoal()::decrement);
  }

  public Command setHoodAngle(Rotation2d angle) {
    return setGoal(ShooterGoal.IDLE).andThen(Commands.runOnce(() -> hood.setPositionGoal(angle)));
  }

  public void setHoodGains(Gains hoodGains) {
    hood.setGains(hoodGains);
  }

  public void setHoodConstraints(AngularPositionConstraints constraints) {
    hood.setProfile(constraints);
  }

  public void setFlywheelGains(Gains flywheelGains) {
    flywheel.updateGains(flywheelGains, GainSlot.ZERO);
  }

  public void setFlywheelConstraints(AngularVelocityConstraints constraints) {
    flywheel.updateConstraints(constraints);
  }

  public Command setFlywheelVelocity(AngularVelocity velocity) {
    return setGoal(ShooterGoal.IDLE)
        .andThen(Commands.runOnce(() -> flywheel.setVelocityGoal(velocity)));
  }
}
