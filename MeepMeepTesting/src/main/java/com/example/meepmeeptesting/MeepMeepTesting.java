package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11)
                .build();


        Pose2d initialPose = new Pose2d(-18.5,-63, Math.toRadians(90));

        Pose2d scoreChamber = new Pose2d(-8,-31, Math.toRadians(90));

//        Pose2d avoidSubmersibleGusset = new Pose2d(-35, -35, Math.toRadians(150));

//        Pose2d scoreRedBasketLeft = new Pose2d(-41,-60, Math.toRadians(180));

        Pose2d scoreHighBasket = new Pose2d(-48, -48, Math.toRadians(225));

        Pose2d firstSpikeMark = new Pose2d(-35, -35, Math.toRadians(150));

        Pose2d secondSpikeMark = new Pose2d(-45, -35, Math.toRadians(150));

        Pose2d parkNearSubmersible = new Pose2d(-24, -12.5, 0);

        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(parkNearSubmersible, Math.toRadians(0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}