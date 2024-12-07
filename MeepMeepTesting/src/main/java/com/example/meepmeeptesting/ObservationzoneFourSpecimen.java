package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
        Pose2d secondSpikeMark = new Pose2d(48, -30, Math.toRadians(300));
        Pose2d thirdSpikeMark = new Pose2d(68, -24, Math.toRadians(270));
        Pose2d observationZonePose = new Pose2d(47, -55, Math.toRadians(270));
        Pose2d secondObservationZonePose = new Pose2d(57, -55, Math.toRadians(270));
        Pose2d thirdObservationZonePose = new Pose2d(67, -55, Math.toRadians(270));
        Pose2d preAcquireSpecimen = new Pose2d(47, -55, Math.toRadians(270));
        Pose2d acquireSpecimenPose = new Pose2d(47, initialPose.position.y, Math.toRadians(270));
        Pose2d chamberTwoPose = new Pose2d(5,-27, Math.toRadians(90));
        Pose2d chamberThreePose = new Pose2d(2,-27, Math.toRadians(90));
        Pose2d chamberFourPose = new Pose2d(-1, -27, Math.toRadians(90));
        Pose2d parkOnWall = new Pose2d(47, -65, Math.toRadians(180));

        TrajectoryActionBuilder scoreChamberTab = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90));

        TrajectoryActionBuilder driveToIntermediateTab = drive.actionBuilder(scoreChamber)
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(intermediatePose,Math.toRadians(45));

        TrajectoryActionBuilder firstSpikeMarkTab = drive.actionBuilder(scoreChamber)
//                .setTangent(Math.toRadians(300))
                .setReversed(true)
                .splineToLinearHeading(intermediatePose, Math.toRadians(0))
                .splineToSplineHeading(firstSpikeMark, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(observationZonePose,Math.toRadians(270))
//                .setTangent(Math.toRadians(90))
                .setReversed(true)
                .splineToSplineHeading(secondSpikeMark, Math.toRadians(270))
                .splineToSplineHeading(acquireSpecimenPose, Math.toRadians(270));

        TrajectoryActionBuilder firstObservationTab = drive.actionBuilder(firstSpikeMark)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(observationZonePose, Math.toRadians(270));
//                .lineToY(observationZonePose.position.y);


        TrajectoryActionBuilder secondSpikeMarkTab = drive.actionBuilder(observationZonePose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(0));

        TrajectoryActionBuilder thirdSpikeMarkTab = drive.actionBuilder(secondObservationZonePose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(thirdSpikeMark, Math.toRadians(0));

        TrajectoryActionBuilder secondObservationZoneTab = drive.actionBuilder(secondSpikeMark)
                .lineToY(secondObservationZonePose.position.y);

        TrajectoryActionBuilder driveBackToObservationZoneTab = drive.actionBuilder(secondObservationZonePose)
                .strafeTo(new Vector2d(observationZonePose.position.x, observationZonePose.position.y));

        TrajectoryActionBuilder strafeToObservationPoseTab = drive.actionBuilder(secondObservationZonePose)
                .strafeToLinearHeading(new Vector2d(observationZonePose.position.x, observationZonePose.position.y), Math.toRadians(270));
//                .strafeTo(new Vector2d(observationZonePose.position.x, observationZonePose.position.y));

        TrajectoryActionBuilder acquireSecondSpecimenTab = drive.actionBuilder(secondObservationZonePose)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270));

//        TrajectoryActionBuilder turnToSpecimenTab = drive.actionBuilder(observationZonePose)
//                .turnTo(Math.toRadians(270))
//                .lineToY(parkOnWall.position.y);

        TrajectoryActionBuilder secondScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToSplineHeading(chamberTwoPose, Math.toRadians(90),null, new ProfileAccelConstraint(-40,40));

        TrajectoryActionBuilder thirdObservationZoneTab = drive.actionBuilder(thirdSpikeMark)
                .lineToY(thirdObservationZonePose.position.y);

        TrajectoryActionBuilder acquireThirdSpecimen = drive.actionBuilder(chamberTwoPose)
                .setReversed(true)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270));

        TrajectoryActionBuilder acquireFourthSpecimen = drive.actionBuilder(chamberThreePose)
                .setReversed(true)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270));

        TrajectoryActionBuilder thirdScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToLinearHeading(chamberThreePose, Math.toRadians(90),null, new ProfileAccelConstraint(-40,40));
        TrajectoryActionBuilder fouthScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToLinearHeading(chamberFourPose, Math.toRadians(90),null, new ProfileAccelConstraint(-40,40));

        TrajectoryActionBuilder parkInObservationZone = drive.actionBuilder(chamberFourPose)
                .setReversed(true)
                .splineToLinearHeading(parkOnWall, Math.toRadians(270));

// 4 specimens (1 pre-load, 1 pit-load, 2 from spikemarks)

        myBot.runAction(new SequentialAction(
                scoreChamberTab.build(),
                firstSpikeMarkTab.build(),
                secondScoreChamberTab.build(),
                acquireThirdSpecimen.build(),
                thirdScoreChamberTab.build(),
                acquireFourthSpecimen.build(),
                fouthScoreChamberTab.build()
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}