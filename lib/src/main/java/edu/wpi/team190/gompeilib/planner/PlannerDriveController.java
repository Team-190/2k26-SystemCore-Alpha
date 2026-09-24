import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.ProfiledPIDController;

public class PlannerDriveController {
    private final PIDController m_xController;
	private final PIDController m_yController;
	private final ProfiledPIDController m_thetaController;

    public Planner(PIDController pidX, PIDController pidY, ProfiledPIDController pidTheta) {
        
m_xController = pidY;
        m_yController = pidX;
        m_thetaController = pidTheta;
    }
}