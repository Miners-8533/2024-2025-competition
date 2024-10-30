package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;

public class PoseConfig {
    public Pose2d getInitialPose(){
        return new Pose2d(16,-62, Math.toRadians(90));
    }
    public Pose2d getChamberPose(){return new Pose2d(8,-31, Math.toRadians(90));}
    public Pose2d getChamberTwoPose(){
        return new Pose2d(5,-31, Math.toRadians(90));
    }
    public Pose2d getChamberThreePose(){
        return new Pose2d(2,-31, Math.toRadians(90));
    }
    public Pose2d getFirstSpikeMarkPose(){
        return new Pose2d(35, -35, Math.toRadians(30));
    }
    public Pose2d getSecondSpikeMarkPose(){
        return new Pose2d(45, -35, Math.toRadians(30));
    }
    public Pose2d getObservationZonePose(){
        return new Pose2d(47, -58, Math.toRadians(315));
    }
    public Pose2d getBasketPose(){
        return new Pose2d(-48, -48, Math.toRadians(225));
    }
    public Pose2d getParkSubmersiblePose(){
        return new Pose2d(-24, -6.5, Math.toRadians(180));
    }
}
