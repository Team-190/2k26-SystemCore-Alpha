package edu.wpi.team190.gompeilib.io.motor.inputs;

import static java.util.Map.entry;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import edu.wpi.team190.gompeilib.io.IOTestSupport;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class MotorTelemetryTest {

  /** Which {@link MotorTelemetry} fields each TalonFX status signal fills. */
  public static final Map<String, List<String>> TALONFX_SIGNAL_TO_FIELDS =
      Map.ofEntries(
          entry("Version", List.of("firmwareVersion")),
          entry("IsProLicensed", List.of("isProLicensed")),
          entry("Position", List.of("position")),
          entry("Velocity", List.of("velocity")),
          entry("Acceleration", List.of("acceleration")),
          entry("RotorPosition", List.of("rotorPosition")),
          entry("RotorVelocity", List.of("rotorVelocity")),
          entry("MotorVoltage", List.of("appliedVoltage")),
          entry("SupplyVoltage", List.of("supplyVoltage")),
          entry("DutyCycle", List.of("dutyCycle")),
          entry("SupplyCurrent", List.of("supplyCurrent")),
          entry("StatorCurrent", List.of("outputCurrent")),
          entry("TorqueCurrent", List.of("torqueCurrent")),
          entry("DeviceTemp", List.of("deviceTemperature")),
          entry("ProcessorTemp", List.of("processorTemperature")),
          entry("AncillaryDeviceTemp", List.of("ancillaryDeviceTemperature")),
          entry("AppliedRotorPolarity", List.of("rotorPolarity")),
          entry("ConnectedMotor", List.of("connectedMotor")),
          entry("MotorKT", List.of("motorKT")),
          entry("MotorKV", List.of("motorKV")),
          entry("MotorStallCurrent", List.of("motorStallCurrent")),
          entry("RobotEnable", List.of("robotEnabled")),
          entry("DeviceEnable", List.of("deviceEnabled")),
          entry("ControlMode", List.of("controlMode", "focActive")),
          entry("MotorOutputStatus", List.of("motorOutputStatus")),
          entry("BridgeOutput", List.of("bridgeOutput")),
          entry("ClosedLoopSlot", List.of("gainSlot")),
          entry(
              "ClosedLoopReference",
              List.of("closedLoopPositionReference", "closedLoopVelocityReference")),
          entry(
              "ClosedLoopReferenceSlope",
              List.of("closedLoopPositionReferenceSlope", "closedLoopVelocityReferenceSlope")),
          entry("ClosedLoopError", List.of("closedLoopPositionError", "closedLoopVelocityError")),
          entry("ClosedLoopProportionalOutput", List.of("closedLoopProportionalOutput")),
          entry("ClosedLoopIntegratedOutput", List.of("closedLoopIntegratedOutput")),
          entry("ClosedLoopDerivativeOutput", List.of("closedLoopDerivativeOutput")),
          entry("ClosedLoopFeedForward", List.of("closedLoopFeedforward")),
          entry("ClosedLoopOutput", List.of("closedLoopOutput")),
          entry("MotionMagicAtTarget", List.of("profileAtTarget")),
          entry("MotionMagicIsRunning", List.of("profileRunning")),
          entry(
              "DifferentialControlMode",
              List.of("differentialControlMode", "differentialFocActive")),
          entry("DifferentialAveragePosition", List.of("differentialAveragePosition")),
          entry("DifferentialAverageVelocity", List.of("differentialAverageVelocity")),
          entry("DifferentialDifferencePosition", List.of("differentialDifferencePosition")),
          entry("DifferentialDifferenceVelocity", List.of("differentialDifferenceVelocity")),
          entry("DifferentialClosedLoopSlot", List.of("differentialGainSlot")),
          entry("DifferentialOutput", List.of("differentialOutput")),
          entry(
              "DifferentialClosedLoopProportionalOutput",
              List.of("differentialClosedLoopProportionalOutput")),
          entry(
              "DifferentialClosedLoopIntegratedOutput",
              List.of("differentialClosedLoopIntegratedOutput")),
          entry(
              "DifferentialClosedLoopDerivativeOutput",
              List.of("differentialClosedLoopDerivativeOutput")),
          entry("DifferentialClosedLoopFeedForward", List.of("differentialClosedLoopFeedforward")),
          entry("DifferentialClosedLoopOutput", List.of("differentialClosedLoopOutput")),
          entry(
              "DifferentialClosedLoopReference",
              List.of(
                  "differentialClosedLoopPositionReference",
                  "differentialClosedLoopVelocityReference")),
          entry(
              "DifferentialClosedLoopReferenceSlope",
              List.of(
                  "differentialClosedLoopPositionReferenceSlope",
                  "differentialClosedLoopVelocityReferenceSlope")),
          entry(
              "DifferentialClosedLoopError",
              List.of(
                  "differentialClosedLoopPositionError", "differentialClosedLoopVelocityError")),
          entry("ForwardLimit", List.of("forwardLimitSwitchAsserted")),
          entry("ReverseLimit", List.of("reverseLimitSwitchAsserted")));

  /** {@link MotorTelemetry} fields no single TalonFX status signal fills, and why. */
  public static final Map<String, String> TALONFX_FIELDS_WITHOUT_SIGNAL =
      IOTestSupport.with(
          IOTestSupport.COMMON_FIELDS_WITHOUT_SIGNAL,
          Map.of(
              "rotorAcceleration", "TalonFX publishes no rotor acceleration",
              "odometryTimestamps", "filled from the high-frequency odometry thread",
              "odometryPositions", "filled from the high-frequency odometry thread"));

  @Test
  public void testDefaultValues() throws IllegalAccessException {
    IOTestSupport.assertZeroDefaults(new MotorTelemetry());
  }

  @Test
  public void testCoversEveryTalonFXSignal() {
    IOTestSupport.assertTelemetryCoverage(
        CoreTalonFX.class,
        MotorTelemetry.class,
        TALONFX_SIGNAL_TO_FIELDS,
        IOTestSupport.COMMON_UNMAPPED_SIGNALS,
        TALONFX_FIELDS_WITHOUT_SIGNAL);
  }
}
