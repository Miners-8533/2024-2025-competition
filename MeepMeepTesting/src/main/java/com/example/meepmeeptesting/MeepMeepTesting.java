package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {


        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d initialPose = new Pose2d(-24,-63, Math.toRadians(90));
//        Pose2d initialPose = new Pose2d(0,0,0);

        Pose2d scoreChamber = new Pose2d(-8,-31, Math.toRadians(90));

        Pose2d safeChamber = new Pose2d(-8, -60, Math.toRadians(90));

        Pose2d scoreRedBasketLeft = new Pose2d(-41,-60, Math.toRadians(180));

        Pose2d firstSpikeMark = new Pose2d(-41,-22.25, Math.toRadians(180));

        Pose2d stepSix = new Pose2d(-51.75,-25.75, Math.toRadians(180));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialPose)
                        .splineToLinearHeading(scoreChamber, Math.toRadians(90))
                        .waitSeconds(2)
                        .setReversed(true)
                        .splineToLinearHeading(scoreRedBasketLeft, Math.toRadians(180))
                        .waitSeconds(2)
                        .splineToLinearHeading(firstSpikeMark, Math.toRadians(180))
                        .waitSeconds(2)
                        .splineToLinearHeading(scoreRedBasketLeft, Math.toRadians(180))
                        .build());

        // The heading interpolates to the heading specified in `endPose`.
        // Setting `endTangent` affects the shape of the spline path itself.

//        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11)
//                .setColorScheme(new ColorSchemeBlueLight())
//                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialPose)
//                                .splineToLinearHeading(scoreChamber, Math.toRadians(180))
//                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
//                .addEntity(myBot2)
                .start();
    }
}