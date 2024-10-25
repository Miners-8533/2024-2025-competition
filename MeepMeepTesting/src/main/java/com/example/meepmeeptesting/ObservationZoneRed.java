package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ObservationZoneRed {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 11)
                .build();


        Pose2d initialPose = new Pose2d(16,-63, Math.toRadians(90));

        Pose2d scoreChamber = new Pose2d(8,-31, Math.toRadians(90));

        Pose2d firstSpikeMark = new Pose2d(35, -35, Math.toRadians(30));

        Pose2d sampleToObservationZone = new Pose2d(47, -58, Math.toRadians(315));

        Pose2d secondSpikeMark = new Pose2d(45, -35, Math.toRadians(30));

        Pose2d aquireSpecimen = new Pose2d(47, -63, Math.toRadians(270));

        Pose2d scoreChamberTwo= new Pose2d(5,-31, Math.toRadians(90));

        Pose2d scoreChamberThree = new Pose2d(2,-31, Math.toRadians(90));


        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(sampleToObservationZone, Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(sampleToObservationZone, Math.toRadians(180))
                .waitSeconds(2)
                .turnTo(Math.toRadians(270))
                .lineToY(initialPose.position.y)
                .waitSeconds(2)
                .splineToLinearHeading(scoreChamberTwo, Math.toRadians(0))
                .waitSeconds(2)
                .splineToLinearHeading(sampleToObservationZone, Math.toRadians(180))
                .turnTo(Math.toRadians(270))
                .lineToY(initialPose.position.y)
                .waitSeconds(2)
                .splineToLinearHeading(scoreChamberThree, Math.toRadians(0))
                .waitSeconds(2)
                .splineToLinearHeading(sampleToObservationZone, Math.toRadians(180))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}