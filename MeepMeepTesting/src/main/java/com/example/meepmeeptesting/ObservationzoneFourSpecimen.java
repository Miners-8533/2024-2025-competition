package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ObservationzoneFourSpecimen {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 30, Math.toRadians(360), 0.666, 11)
                .build();

        DriveShim drive = myBot.getDrive();

        Pose2d initialPose = new Pose2d(16,-62, Math.toRadians(90));
        Pose2d scoreChamber = new Pose2d(8,-27, Math.toRadians(90));
        Pose2d intermediatePose = new Pose2d(30, -36,Math.toRadians(30));
        Pose2d firstSpikeMark = new Pose2d(38, -30, Math.toRadians(300));
        Pose2d secondSpikeMark = new Pose2d(45, -23, Math.toRadians(270));
//        Pose2d secondSpikeMarkIntermediate = new Pose2d(51, -23, Math.toRadians(270));
        Pose2d observationZonePose = new Pose2d(42, -58, Math.toRadians(270));
        Pose2d secondObservationZonePose = new Pose2d(52, -65, Math.toRadians(270));
        Pose2d acquireSpecimenPose = new Pose2d(41, -65, Math.toRadians(270));
        Pose2d chamberTwoPose = new Pose2d(7,-22, Math.toRadians(90));
        Pose2d chamberThreePose = new Pose2d(5.5,-22, Math.toRadians(90));
        Pose2d chamberFourPose = new Pose2d(3.5, -22, Math.toRadians(90));
        Pose2d parkChamberPose = new Pose2d(-10, -21,Math.toRadians(90));

        TrajectoryActionBuilder scoreChamberTab = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder firstSpikeMarkTab = drive.actionBuilder(scoreChamber)
                .setReversed(true)
                .splineToLinearHeading(intermediatePose, Math.toRadians(0))
                .splineToSplineHeading(firstSpikeMark, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(observationZonePose,Math.toRadians(270))
                .setReversed(true)
                .splineToConstantHeading(secondSpikeMark.position, Math.toRadians(0))
//                .splineToSplineHeading(secondSpikeMarkIntermediate, Math.toRadians(0))
                .splineToSplineHeading(secondObservationZonePose, Math.toRadians(270),null, new ProfileAccelConstraint(-30,30));


        TrajectoryActionBuilder secondScoreChamberTab = drive.actionBuilder(secondObservationZonePose)
                .setReversed(true)
                .splineToSplineHeading(chamberTwoPose, Math.toRadians(90), null, new ProfileAccelConstraint(-40,40));

        TrajectoryActionBuilder acquireThirdSpecimen = drive.actionBuilder(chamberTwoPose)
                .setReversed(true)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270),null, new ProfileAccelConstraint(-35, 50));

        TrajectoryActionBuilder acquireFourthSpecimen = drive.actionBuilder(chamberThreePose)
                .setReversed(true)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270),null, new ProfileAccelConstraint(-35, 50));

        TrajectoryActionBuilder thirdScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToSplineHeading(chamberThreePose, Math.toRadians(90), null, new ProfileAccelConstraint(-35,40));

        TrajectoryActionBuilder fourthScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToSplineHeading(chamberFourPose, Math.toRadians(90),null, new ProfileAccelConstraint(-35,40));

// 4 specimens (1 pre-load, 1 pit-load, 2 from spikemarks)

        myBot.runAction(new SequentialAction(
                scoreChamberTab.build(),
                firstSpikeMarkTab.build(),
                secondScoreChamberTab.build(),
                acquireThirdSpecimen.build(),
                thirdScoreChamberTab.build(),
                acquireFourthSpecimen.build(),
                fourthScoreChamberTab.build()
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}