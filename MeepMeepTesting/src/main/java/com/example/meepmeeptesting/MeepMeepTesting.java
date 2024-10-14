package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d initialPose = new Pose2d(-24,-65, Math.toRadians(90));
//        Pose2d initialPose = new Pose2d(0,0,0);

        Pose2d scoreChamber = new Pose2d(-8,-31, Math.toRadians(90));

        Pose2d safeChamber = new Pose2d(-8, -60, Math.toRadians(90));

        Pose2d stepThree = new Pose2d(-41,-65.25, Math.toRadians(180));

        Pose2d stepFour = new Pose2d(-41,-22.25, Math.toRadians(180));

        Pose2d stepSix = new Pose2d(-51.75,-25.75, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialPose)
                        .splineToLinearHeading(scoreChamber, Math.toRadians(90))
                        .splineToLinearHeading(safeChamber, Math.toRadians(90))
                        .splineToLinearHeading(stepThree, Math.toRadians(180))
//                        .splineToLinearHeading(stepFour, Math.toRadians(180))
//                        .splineToLinearHeading(stepThree, Math.toRadians(180))
//                        .splineToLinearHeading(stepSix, Math.toRadians(180))
//                        .splineToLinearHeading(stepFour, Math.toRadians(180))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}