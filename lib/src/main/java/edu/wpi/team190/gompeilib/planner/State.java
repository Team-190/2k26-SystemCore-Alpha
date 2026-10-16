package edu.wpi.team190.gompeilib.planner;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public record State(Pose2d pose, ChassisVelocities velocity, double distance) {}

// this holds the necessary information for any point along the path
