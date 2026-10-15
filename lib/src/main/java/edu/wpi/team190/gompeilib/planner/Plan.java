package edu.wpi.team190.gompeilib.planner;

import java.util.ArrayList;
import java.util.List;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;

public class Plan{

    public ArrayList<Pose2d> combinedPath = new ArrayList<Pose2d>();

    public Plan(ArrayList<Pose2d>... paths){

        for (ArrayList<Pose2d> path : paths) {
            combinedPath.addAll(path);
        }
    }

    public ArrayList<State> getPlan(){

        for (int i = 0; i < combinedPath.size()-1; i++) {
            Pose2d currentPose = combinedPath.get(i);
            Pose2d nextPose = combinedPath.get(i+1);
            double deltaX = currentPose.getTranslation().getX()-nextPose.getTranslation().getX();
            double deltaY = currentPose.getTranslation().getY()-nextPose.getTranslation().getY();
            double rads = currentPose.getRotation().getRadians()-nextPose.getRotation().getRadians();
            ChassisVelocities vel = new ChassisVelocities(deltaX,deltaY,rads);
            
        }
        return null;
    }
}
//puts together the paths and generates states along the path
