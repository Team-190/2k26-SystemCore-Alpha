package edu.wpi.team190.gompeilib.core.planner;

import org.wpilib.math.kinematics.ChassisVelocities;

import org.wpilib.math.geometry.Pose2d;

public record States(Pose2d pose, ChassisVelocities velocities) {}
