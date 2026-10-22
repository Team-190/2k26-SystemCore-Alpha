package edu.wpi.team190.gompeilib.robot.subsystems.vision.camera;

import org.wpilib.math.util.Units;

public enum CameraType {
  LIMELIGHT_2_PLUS(
      Limelight2PlusConstants.HORIZONTAL_FOV,
      Limelight2PlusConstants.VERTICAL_FOV,
      Limelight2PlusConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight2PlusConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight2PlusConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT),
  LIMELIGHT_3(
      Limelight3Constants.HORIZONTAL_FOV,
      Limelight3Constants.VERTICAL_FOV,
      Limelight3Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight3Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight3Constants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT),
  LIMELIGHT_3G(
      Limelight3GConstants.HORIZONTAL_FOV,
      Limelight3GConstants.VERTICAL_FOV,
      Limelight3GConstants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight3GConstants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight3GConstants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT),
  LIMELIGHT_4(
      Limelight4Constants.HORIZONTAL_FOV,
      Limelight4Constants.VERTICAL_FOV,
      Limelight4Constants.MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight4Constants.MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT,
      Limelight4Constants.MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT),
  THRIFTYCAM(
      ThriftyCamConstants.HORIZONTAL_FOV,
      ThriftyCamConstants.VERTICAL_FOV,
      ThriftyCamConstants.SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      ThriftyCamConstants.MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT,
      ThriftyCamConstants.THETA_STANDARD_DEVIATION_COEFFICIENT),
  DEFAULT();

  public final double horizontalFOV;
  public final double verticalFOV;
  public final double megatagXY;
  public final double megatag2XY;
  public final double theta;

  @Deprecated public final double primaryXYStandardDeviationCoefficient;
  @Deprecated public final double secondaryXYStandardDeviationCoefficient;
  @Deprecated public final double primaryThetaStandardDeviationCoefficient;

  private CameraType(
      double horizontalFOV, double verticalFOV, double megatagXY, double megatag2XY, double theta) {
    this.horizontalFOV = horizontalFOV;
    this.verticalFOV = verticalFOV;
    this.megatagXY = megatagXY;
    this.megatag2XY = megatag2XY;
    this.theta = theta;
    this.primaryXYStandardDeviationCoefficient = megatag2XY;
    this.secondaryXYStandardDeviationCoefficient = megatagXY;
    this.primaryThetaStandardDeviationCoefficient = theta;
  }

  private CameraType() {
    this.horizontalFOV = 0.0;
    this.verticalFOV = 0.0;
    this.megatagXY = 0.0;
    this.megatag2XY = 0.0;
    this.theta = 0.0;
    this.primaryXYStandardDeviationCoefficient = 0.0;
    this.secondaryXYStandardDeviationCoefficient = 0.0;
    this.primaryThetaStandardDeviationCoefficient = 0.0;
  }

  public static final double BLINK_TIME = 0.067;

  private static class Limelight2PlusConstants {
    private static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    private static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    private static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  private static class Limelight3Constants {
    private static final double HORIZONTAL_FOV = Units.degreesToRadians(62.5);
    private static final double VERTICAL_FOV = Units.degreesToRadians(48.9);
    private static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.1;
  }

  private static class Limelight3GConstants {
    private static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    private static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    private static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    private static final double MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  private static class Limelight4Constants {
    private static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    private static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    private static final double MEGATAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    private static final double MEGATAG_THETA_STANDARD_DEVIATION_COEFFICIENT = 10;
    private static final double MEGATAG_2_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }

  private static class ThriftyCamConstants {
    private static final double HORIZONTAL_FOV = Units.degreesToRadians(82.0);
    private static final double VERTICAL_FOV = Units.degreesToRadians(46.2);
    private static final double SINGLETAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.05;
    private static final double THETA_STANDARD_DEVIATION_COEFFICIENT = 0.1;
    private static final double MULTITAG_XY_STANDARD_DEVIATION_COEFFICIENT = 0.00015;
  }
}
