package edu.wpi.team190.gompeilib.planner;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class PlannerDriveController {
    private Pose2d m_poseError = new Pose2d();
    private Rotation2d m_rotationError = new Rotation2d();
    private Pose2d m_poseTolerance = new Pose2d();
    private Boolean m_firstRun = true;

    private final PIDController m_xController;
    private final PIDController m_yController;
    private final ProfiledPIDController m_thetaController;

    public PlannerDriveController(PlannerControllerConfig config) {
        m_xController = config.m_xController();
        m_yController = config.m_yController();
        m_thetaController = config.m_thetaController();
        m_thetaController.enableContinuousInput(0, Math.PI * 2);
    }

    public boolean atReference() {
        final Translation2d eTransate = m_poseError.getTranslation();
        final Rotation2d eRotate = m_rotationError;
        final Translation2d tolTranslate = m_poseTolerance.getTranslation();
        final Rotation2d tolRotate = m_poseTolerance.getRotation();
        return Math.abs(eTransate.getX()) < tolTranslate.getX()
                && Math.abs(eTransate.getY()) < tolTranslate.getY()
                && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
    }

    public void setTolerance(Pose2d tolerance) {
        m_poseTolerance = tolerance;
    }

    public ChassisVelocities calculate(
            Pose2d currentPose,
            Pose2d trajectoryPose,
            double desiredLinearVelocity // m/s velocity at time of call
    ) {
        if (m_firstRun) {
            m_thetaController.reset(currentPose.getTranslation().getAngle().getRadians());
            m_firstRun = false;
        }

        // calculate feedforward velocities
        double xFF = desiredLinearVelocity * trajectoryPose.getRotation().getCos()
        double yFF = desiredLinearVelocity * trajectoryPose.getRotation().getSin();
        double thetaFF = m_thetaController.calculate(
                currentPose.getRotation().getRadians(),
                trajectoryPose.getRotation().getRadians());

        m_poseError = trajectoryPose.relativeTo(currentPose);
        m_rotationError = trajectoryPose.getRotation().minus(currentPose.getRotation());

        double xFeedBack = m_xController.calculate(currentPose.getX());
        double yFeedback = m_yController.calculate(currentPose.getY());

        return new ChassisVelocities(xFF, yFF, thetaFF).toRobotRelative(currentPose.getRotation());
    }
}
