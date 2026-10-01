package first.robot.v3_Horse_CV2_TR.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import first.robot.util.AllianceFlipUtil;
import first.robot.v3_Horse_CV2_TR.V3_Horse_CV2_TRRobotState;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloor;
import first.robot.v3_Horse_CV2_TR.subsystems.rollerfloor.V3_Horse_CV2_TR_RollerFloorConstants.RollerFloorState;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooter;
import first.robot.v3_Horse_CV2_TR.subsystems.shooter.V3_Horse_CV2_TRShooterConstants.ShooterGoal;

public class V3_Horse_CV2_TRCompositeCommands {

    public static Command resetHeading(
            SwerveDrive drive,
            Consumer<Pose2d> resetHeadingConsumer,
            Supplier<Translation2d> currentRobotTranslation) {
        return Commands.runOnce(
                () -> {
                    resetHeadingConsumer.accept(
                            new Pose2d(
                                    currentRobotTranslation.get(), AllianceFlipUtil.apply(new Rotation2d())));
                })
                .ignoringDisable(true);
    }

    public static Command updateCurrentLimits(
            SwerveDrive drive, double driveCurrentLimit, double turnCurrentLimit) {
        return Commands.runOnce(() -> drive.updateCurrentLimits(driveCurrentLimit, turnCurrentLimit))
                .ignoringDisable(true);
    }

    public static Command scoreOrFeedCommand(
            V3_Horse_CV2_TR_RollerFloor rollerFloor, V3_Horse_CV2_TRShooter shooter) {
        return Commands.sequence(
                shooter.setGoal(
                        () -> (V3_Horse_CV2_TRRobotState.isInAllianceZone()
                                ? ShooterGoal.SCORE
                                : ShooterGoal.FEED)),
                shooter.waitUntilAtGoal(),
                rollerFloor.setState(RollerFloorState.RUN));
    }

    public static Command stopShooter(V3_Horse_CV2_TR_RollerFloor rollerFloor, V3_Horse_CV2_TRShooter shooter) {
        return Commands.parallel(shooter.setGoal(ShooterGoal.STOP), rollerFloor.setState(RollerFloorState.STOP));
    }

    public static Command shootWithAgitateCommand(V3_Horse_CV2_TR_RollerFloor rollerFloor,
            V3_Horse_CV2_TRShooter shooter, V3_Horse_CV2_TR_Intake intake) {
        return Commands.parallel(V3_Horse_CV2_TRCompositeCommands.scoreOrFeedCommand(rollerFloor, shooter),
                intake.setState(intakeState.AGITATE));
    }

    public static Command bumpShotCommand(V3_Horse_CV2_TR_RollerFloor rollerFloor, V3_Horse_CV2_TRShooter shooter,
            V3_Horse_CV2_TR_Intake intake) {

        return Commands.parallel(
                Commands.sequence(
                        shooter.setGoal(ShooterGoal.BUMP_SHOT),
                        shooter.waitUntilAtGoal(),
                        rollerFloor.setState(null)),

                intake.setState(intakeState.AGITATE));

    }

    public static Command trenchShotCommand(V3_Horse_CV2_TR_RollerFloor rollerFloor, V3_Horse_CV2_TRShooter shooter,
            V3_Horse_CV2_TR_Intake intake) {

        return Commands.parallel(
                Commands.sequence(
                        shooter.setGoal(ShooterGoal.TRENCH_SHOT),
                        shooter.waitUntilAtGoal(),
                        rollerFloor.setState(null)),

                intake.setState(intakeState.AGITATE));

    }

    public static Command farShotCommand(V3_Horse_CV2_TR_RollerFloor rollerFloor, V3_Horse_CV2_TRShooter shooter,
            V3_Horse_CV2_TR_Intake intake) {

        return Commands.parallel(
                Commands.sequence(
                        shooter.setGoal(ShooterGoal.FAR_SHOT),
                        shooter.waitUntilAtGoal(),
                        rollerFloor.setState(null)),

                intake.setState(intakeState.AGITATE));

    }

}
