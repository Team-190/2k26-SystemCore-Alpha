package edu.wpi.team190.gompeilib.planner;

import java.util.Optional;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

//Impliment ts later get basic stuff in im just bored 
public class PathNode {
    public Optional<Rotation2d> rotation;
    public Optional<Translation2d> translation;
    public double desiredLinearVelocity;

    public PathNode(Pose2d pose, double desiredLinearVelocity) {
        rotation = Optional.of(pose.getRotation());
        translation = Optional.of(pose.getTranslation());
        this.desiredLinearVelocity = desiredLinearVelocity;
    }

    public PathNode(Rotation2d rotation2d, double desiredLinearVelocity) {
        rotation = Optional.of(rotation2d);
        translation = Optional.ofNullable(null);
        this.desiredLinearVelocity = desiredLinearVelocity;
    }

    public PathNode(Translation2d translation2d, double desiredLinearVelocity) {
        rotation = Optional.ofNullable(null);
        translation = Optional.of(translation2d);
        this.desiredLinearVelocity = desiredLinearVelocity;
    }
}