package first.robot.v3_Horse_CV2_TR.subsystems.shooter;

import static org.wpilib.units.Units.*;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Voltage;
import org.wpilib.units.measure.Current;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.RobotState;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Setpoint;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheel;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHood;
import edu.wpi.team190.gompeilib.subsystems.generic.hood.GenericHoodIO;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants.HoodGoal;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class V3_Horse_CV2_TRShooter extends SubsystemBase {

    private final GenericFlywheel flywheel;

    private final GenericHood hood;
    private HoodGoal hoodGoal;

    public V3_Horse_CV2_TRShooter(GenericFlywheelIO flywheelIO, GenericHoodIO hoodIO) {

        Setpoint<AngularVelocityUnit> flywheelVelocitySetpoint = new Setpoint<AngularVelocityUnit>(
                RadiansPerSecond.of(0.0), V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS.velocityOffsetStep);
        Setpoint<VoltageUnit> flywheelVoltageSetpoint = new Setpoint<VoltageUnit>(Volts.of(0.0),
                V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS.voltageOffsetStep);

        Setpoint<AngleUnit> hoodAngleSetpoint = new Setpoint<AngleUnit>(Radians.of(0.0),
                V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS.offsetStep);
        Setpoint<VoltageUnit> hoodVoltageSetpoint = new Setpoint<VoltageUnit>(Volts.of(0.0),
                V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS.voltageStep);

        this.flywheel = new GenericFlywheel(flywheelIO, this, V3_Horse_CV2_TRShooterConstants.SHOOT_CONSTANTS, "",
                flywheelVelocitySetpoint, flywheelVoltageSetpoint);
        this.hood = new GenericHood(hoodIO, this, V3_Horse_CV2_TRShooterConstants.HOOD_CONSTANTS, "", hoodAngleSetpoint,
                hoodVoltageSetpoint);
        this.hoodGoal = HoodGoal.STOW;

    }

    @Trace
    public void periodic() {

        hood.periodic();
        flywheel.periodic();

        Logger.recordOutput("Shooter/Hood/Goal", hoodGoal);

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

    }

    public Command setHoodGoal(HoodGoal goal) {
        return Commands.runOnce(
                () -> {
                    hoodGoal = goal;
                    hood.setPositionGoal(getHoodGoal(goal));
                });

    }

    private Rotation2d getHoodGoal(HoodGoal goal) {
        Rotation2d rotation = switch (goal) {
            case SCORE -> V3_Horse_CV2_TRRobotState.getScoreAngle();
            case FEED -> V3_Horse_CV2_TRRobotState.getFeedAngle();
            default -> Rotation2d.kZero;
        };
        Logger.recordOutput("Shooter/Hood/DebugGoal", rotation);
        return rotation;
    }

    public Command setOverrideHoodGoal(Rotation2d position) {
        return Commands.runOnce(() -> hood.setPositionGoal(position))
                .andThen(Commands.runOnce(() -> hoodGoal = HoodGoal.OVERRIDE));
    }

    public Command setHoodVoltage(double volts) {
        return Commands.runOnce(() -> hood.setVoltageGoal(Volts.of(volts)));
    }

    public Command stopHood() {
        return Commands.runOnce(() -> hood.setVoltageGoal(Volts.zero()));
    }

    public Command zeroHood() {
        return hood.resetHoodZero();
    }

    public Command setFlywheelGoal(AngularVelocity velocityGoal) {
        return Commands.runOnce(() -> flywheel.setVelocityGoal(velocityGoal));
    }

    public Command setFlywheelGoal(AngularVelocity velocityGoal, Current feedforward) {
        return Commands.runOnce(() -> flywheel.setVelocityGoal(velocityGoal, feedforward));
    }

    public Command setFlywheelVoltage(double volts) {
        return Commands.runOnce(() -> flywheel.setVoltageGoal(Volts.of(volts)));
    }

    public Command stopFlywheel() {
        return Commands.runOnce(flywheel::stop);
    }

    public Command setGoal(HoodGoal hoodGoal, double velocityRadiansPerSecond) {
        return Commands.parallel(
                setHoodGoal(hoodGoal), setFlywheelGoal(RadiansPerSecond.of(velocityRadiansPerSecond)));
    }

    public Command setGoal(HoodGoal hoodGoal, DoubleSupplier velocityRadiansPerSecond) {
        return Commands.parallel(
                Commands.run(
                        () -> {
                            this.hoodGoal = hoodGoal;
                            hood.setPositionGoal(getHoodGoal(hoodGoal));
                        }),
                Commands.run(
                        () -> flywheel.setVelocityGoal(
                                RadiansPerSecond.of(velocityRadiansPerSecond.getAsDouble()))));
    }

    public boolean atGoal() {
        return hood.atPositionGoal() && flywheel.atVelocityGoal();
    }

    public Command waitUntilAtGoal() {
        return hood.waitUntilAtGoal().alongWith(flywheel.waitUntilAtGoal());
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

}