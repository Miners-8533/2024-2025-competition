package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ObservationZoneRedValidateMenuLogic {
    enum AllianceColor {
        RED,
        BLUE
    }
    enum FieldStartPosition {
        LEFT,
        RIGHT
    }
    static Pose2d getInitialPose(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        double poseX;
        double poseY;
        double poseH;

        if (fieldStartPosition == FieldStartPosition.LEFT){
            poseX = (allianceColor == AllianceColor.RED) ? -16 : 16;
        }
        else {
            poseX = (allianceColor == AllianceColor.RED) ? 16 : -16;
        }

        poseY = (allianceColor == AllianceColor.RED) ? -63 : 63;
        poseH = (allianceColor == AllianceColor.RED) ? 90 : 270;

        return new Pose2d(poseX, poseY, Math.toRadians(poseH));
    }

    static Pose2d getChamberPose(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        double poseX;
        double poseY;
        double poseH;

//        Pose2d scoreChamber = new Pose2d(8,-31, Math.toRadians(90));
        if (fieldStartPosition == FieldStartPosition.LEFT){
            poseX = (allianceColor == AllianceColor.RED) ? -8 : 8;
        }
        else {
            poseX = (allianceColor == AllianceColor.RED) ? 8 : -8;
        }

        poseY = (allianceColor == AllianceColor.RED) ? -31 : 31;
        poseH = (allianceColor == AllianceColor.RED) ? 90 : 270;

        return new Pose2d(poseX, poseY, Math.toRadians(poseH));
    }

    static Pose2d getfirstSpikeMarkPose(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        double poseX;
        double poseY;
        double poseH;

        if (fieldStartPosition == FieldStartPosition.LEFT){
            poseX = (allianceColor == AllianceColor.RED) ? -35 : 35;
            poseH = (allianceColor == AllianceColor.RED) ? 150 : 330;

        }
        else {
            poseX = (allianceColor == AllianceColor.RED) ? 35 : -35;
            poseH = (allianceColor == AllianceColor.RED) ? 30 : 210;
        }

        poseY = (allianceColor == AllianceColor.RED) ? -35 : 35;


        return new Pose2d(poseX, poseY, Math.toRadians(poseH));
    }
    static Pose2d getSecondSpikeMarkPose(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        double poseX;
        double poseY;
        double poseH;

        if (fieldStartPosition == FieldStartPosition.LEFT){
            poseX = (allianceColor == AllianceColor.RED) ? -45 : 45;
            poseH = (allianceColor == AllianceColor.RED) ? 150 : 330;
        }
        else {
            poseX = (allianceColor == AllianceColor.RED) ? 45 : -45;
            poseH = (allianceColor == AllianceColor.RED) ? 30 : 210;
        }

        poseY = (allianceColor == AllianceColor.RED) ? -35 : 35;


        return new Pose2d(poseX, poseY, Math.toRadians(poseH));
    }

    static Vector2d rotate(double x, double y, double theta) {
        return new Vector2d(x * Math.cos(theta) + y * Math.sin(theta),
                -x * Math.sin(theta) + y * Math.cos(theta));
    }

    static Pose2d mirrorPose(double x, double y, double heading) {
        // Flip the X and Y coordinates
        double mirroredX = -x;
        double mirroredY = -y;

        // Adjust the heading to be mirrored
        double mirroredHeading = heading + Math.PI;

        // Normalize the heading to be within [-PI, PI]
        mirroredHeading = (mirroredHeading + Math.PI) % (2 * Math.PI) - Math.PI;

        return new Pose2d(mirroredX, mirroredY, mirroredHeading);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);


        AllianceColor allianceColor = AllianceColor.RED;
        FieldStartPosition fieldStartPosition = FieldStartPosition.RIGHT;


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        Pose2d initialPose = getInitialPose(allianceColor, fieldStartPosition);
//        Pose2d initialPose = new Pose2d(16,-63, Math.toRadians(90));

//        Pose2d scoreChamber = new Pose2d(8,-31, Math.toRadians(90));
        Pose2d scoreChamber = getChamberPose(allianceColor, fieldStartPosition);
//        Pose2d firstSpikeMark = new Pose2d(35, -35, Math.toRadians(30));
        Pose2d firstSpikeMark = getfirstSpikeMarkPose(allianceColor, fieldStartPosition);

        Pose2d sampleToObservationZone = new Pose2d(47, -58, Math.toRadians(315));

//        Pose2d secondSpikeMark = new Pose2d(45, -35, Math.toRadians(30));
        Pose2d secondSpikeMark = getSecondSpikeMarkPose(allianceColor, fieldStartPosition);

        Pose2d aquireSpecimen = new Pose2d(47, -63, Math.toRadians(270));

        Pose2d scoreChamberTwo= new Pose2d(5,-31, Math.toRadians(90));

        Pose2d scoreChamberThree = new Pose2d(2,-31, Math.toRadians(90));

        double rotationBasis = 180;

//        Vector2d rotatedInitialPose = rotate(initialPose.position.x, initialPose.position.y, rotationBasis);
        Pose2d mirroredPose = mirrorPose(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());

        myBot.runAction(myBot.getDrive().actionBuilder(mirroredPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(90))
                .waitSeconds(2)
                .splineToLinearHeading(sampleToObservationZone, Math.toRadians(270))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(sampleToObservationZone, Math.toRadians(180))
                .waitSeconds(2)
                .turnTo(Math.toRadians(270))
                .lineToY(initialPose.position.y)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(scoreChamberTwo, Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(sampleToObservationZone, Math.toRadians(270))
                .turnTo(Math.toRadians(270))
                .lineToY(initialPose.position.y)
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(scoreChamberThree, Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(sampleToObservationZone, Math.toRadians(270))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
