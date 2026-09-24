package edu.wpi.team190.gompeilib.planner;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;

/**
 * Used to store the pid data for the PlannerDriveController class 
 */

public record PlannerControllerConfig(PIDController m_xController, PIDController m_yController,
        ProfiledPIDController m_thetaController) {
}