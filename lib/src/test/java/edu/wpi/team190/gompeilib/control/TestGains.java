package edu.wpi.team190.gompeilib.control;

import static org.wpilib.units.Units.*;

import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.MultUnit;
import org.wpilib.units.VoltageUnit;

/**
 * Unpublished gains for tests, declared in a Phoenix slot's native units (volts per gearbox-output
 * rotation, or per rotation per second) so each test's numbers keep the meaning they had before
 * gains were typed.
 */
public final class TestGains {
  private TestGains() {}

  /**
   * Returns position gains in volts per rotation.
   *
   * @return the gains
   */
  public static Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> position(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    return Gains.angularPosition(Volts)
        .withKP(Volts.per(Rotations).ofNative(kP))
        .withKI(Volts.per(MultUnit.combine(Rotations, Seconds)).ofNative(kI))
        .withKD(Volts.per(RotationsPerSecond).ofNative(kD))
        .withKS(Volts.of(kS))
        .withKV(Volts.per(RotationsPerSecond).ofNative(kV))
        .withKA(Volts.per(RotationsPerSecondPerSecond).ofNative(kA))
        .withKG(Volts.of(kG))
        .build();
  }

  /**
   * Returns position gains in volts per rotation with only {@code kP} set.
   *
   * @return the gains
   */
  public static Gains<VoltageUnit, AngleUnit, AngularVelocityUnit> positionKP(double kP) {
    return position(kP, 0, 0, 0, 0, 0, 0);
  }

  /**
   * Returns velocity gains in volts per rotation per second.
   *
   * @return the gains
   */
  public static Gains<VoltageUnit, AngularVelocityUnit, AngularVelocityUnit> velocity(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {
    return Gains.angularVelocity(Volts)
        .withKP(Volts.per(RotationsPerSecond).ofNative(kP))
        .withKI(Volts.per(MultUnit.combine(RotationsPerSecond, Seconds)).ofNative(kI))
        .withKD(Volts.per(RotationsPerSecondPerSecond).ofNative(kD))
        .withKS(Volts.of(kS))
        .withKV(Volts.per(RotationsPerSecond).ofNative(kV))
        .withKA(Volts.per(RotationsPerSecondPerSecond).ofNative(kA))
        .withKG(Volts.of(kG))
        .build();
  }
}
