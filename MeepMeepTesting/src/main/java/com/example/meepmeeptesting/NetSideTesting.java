package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Robot;

public class NetSideTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 45, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        DriveShim drive = myBot.getDrive();

        Pose2d initialPose = new Pose2d(-16,-62, Math.toRadians(90));
        Pose2d scoreChamber = new Pose2d(-8,-28, Math.toRadians(90));
        Pose2d intermediatePose = new Pose2d(-21, -37,Math.toRadians(120));
        Pose2d scoreHighBasket = new Pose2d(-55, -47, Math.toRadians(225));
        Pose2d firstSpikeMark = new Pose2d(-36, -21, Math.toRadians(180));
        Pose2d secondSpikeMark = new Pose2d(-45.5, -20.5, Math.toRadians(180));
        Pose2d thirdSpikeMark = new Pose2d(-53.5, -20, Math.toRadians(175));
        Pose2d parkNearSubmersible = new Pose2d(-15, -6.5, Math.toRadians(180));

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder driveToIntermediate = drive.actionBuilder(scoreChamber)
                .setReversed(true)
                .splineToLinearHeading(intermediatePose, Math.toRadians(180));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoreChamber)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(intermediatePose, Math.toRadians(180), null, new ProfileAccelConstraint(-30,30))
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(90),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab3  = drive.actionBuilder(firstSpikeMark)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(scoreHighBasket)
                .setTangent(0)
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180),null, new ProfileAccelConstraint(-15,50));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(secondSpikeMark)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180),null, new ProfileAccelConstraint(-40,40));

        TrajectoryActionBuilder tab6 = drive.actionBuilder(scoreHighBasket)
                .setReversed(true)
                .splineToLinearHeading(parkNearSubmersible, Math.toRadians(0),null, new ProfileAccelConstraint(-30,50));

        TrajectoryActionBuilder tab7 = drive.actionBuilder(scoreHighBasket)
                .setTangent(0)
                .splineToLinearHeading(thirdSpikeMark,Math.toRadians(180),null, new ProfileAccelConstraint(-15,50));
// 4 specimens (1 pre-load, 1 pit-load, 2 from spikemarks)

        myBot.runAction(new SequentialAction(
                tab1.build(),
                tab2.build(),
                tab3.build(),
                tab4.build(),
                tab5.build(),
                tab7.build(),
                tab5.build(),
                tab6.build()
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}