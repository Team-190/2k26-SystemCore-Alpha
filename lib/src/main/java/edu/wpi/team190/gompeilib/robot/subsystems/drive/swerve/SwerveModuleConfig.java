package edu.wpi.team190.gompeilib.robot.subsystems.drive.swerve;

import edu.wpi.team190.gompeilib.io.encoder.config.AbsoluteEncoderConfig;
import edu.wpi.team190.gompeilib.io.motor.config.MotorConfig;
import edu.wpi.team190.gompeilib.io.physics.PlantCharacterization;
import lombok.Builder;
import lombok.NonNull;
import org.wpilib.math.geometry.Translation2d;

/**
 * One swerve module's hardware and geometry.
 *
 * @param driveMotor the drive motor; its gear ratio is rotor rotations per wheel rotation and slot
 *     {@code ZERO} holds its velocity gains
 * @param turnMotor the turn motor; its gear ratio is rotor rotations per module rotation and slot
 *     {@code ZERO} holds its position gains
 * @param encoder the absolute encoder on the module's turn axis
 * @param location the module's position relative to the robot's center
 * @param drivePlant how the drive axis responds to its motor at the wheel, for simulation
 * @param turnPlant how the turn axis responds to its motor at the module, for simulation
 */
@Builder(setterPrefix = "with")
public record SwerveModuleConfig(
    @NonNull MotorConfig driveMotor,
    @NonNull MotorConfig turnMotor,
    @NonNull AbsoluteEncoderConfig encoder,
    @NonNull Translation2d location,
    @NonNull PlantCharacterization<?> drivePlant,
    @NonNull PlantCharacterization<?> turnPlant) {}
