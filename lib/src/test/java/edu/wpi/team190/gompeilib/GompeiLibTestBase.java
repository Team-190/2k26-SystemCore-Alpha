package edu.wpi.team190.gompeilib;

import edu.wpi.team190.gompeilib.robot.RobotMode;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.wpilib.hardware.hal.HAL;

/**
 * Base fixture for GompeiLib tests.
 *
 * <p>Initializes the WPILib HAL and sets up the GompeiLib environment in SIM mode before each test,
 * and resets/closes registered AutoCloseable resources and deinitializes GompeiLib after each test.
 */
public abstract class GompeiLibTestBase {
  private final List<AutoCloseable> autoCloseables = new ArrayList<>();

  @BeforeEach
  public void setUpGompeiLib() {
    assert HAL.initialize();
    try {
      GompeiLib.deinit();
    } catch (Exception ignored) {
    }
    GompeiLib.init(RobotMode.SIM, isTuningMode(), loopPeriodSecs());
  }

  @AfterEach
  public void tearDownGompeiLib() {
    for (AutoCloseable resource : autoCloseables) {
      if (resource != null) {
        try {
          resource.close();
        } catch (Exception ignored) {
        }
      }
    }
    autoCloseables.clear();
    try {
      GompeiLib.deinit();
    } catch (Exception ignored) {
    }
  }

  /**
   * Registers an AutoCloseable resource to be closed in @AfterEach.
   *
   * @param resource the resource to close
   * @param <T> type of AutoCloseable
   * @return the registered resource
   */
  public <T extends AutoCloseable> T registerCloseable(T resource) {
    if (resource != null) {
      autoCloseables.add(resource);
    }
    return resource;
  }

  /**
   * Override to customize whether tuning mode is enabled during test setup.
   *
   * @return true if tuning mode is enabled, false otherwise (default: false)
   */
  protected boolean isTuningMode() {
    return false;
  }

  /**
   * Override to customize loop period during test setup.
   *
   * @return loop period in seconds (default: {@link GompeiLib#LOOP_PERIOD_SECS})
   */
  protected double loopPeriodSecs() {
    return GompeiLib.LOOP_PERIOD_SECS;
  }

  /**
   * Re-initializes GompeiLib with specific settings during a test.
   *
   * @param mode robot mode
   * @param isTuning whether tuning mode is enabled
   * @param loopPeriodSecs loop period in seconds
   */
  protected void reinitGompeiLib(RobotMode mode, boolean isTuning, double loopPeriodSecs) {
    try {
      GompeiLib.deinit();
    } catch (Exception ignored) {
    }
    GompeiLib.init(mode, isTuning, loopPeriodSecs);
  }
}
