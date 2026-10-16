package edu.wpi.team190.gompeilib.planner.Containers;

import org.wpilib.math.geometry.Pose2d;

public record PosePlus(Pose2d pose, double vel, double angVel) {}
