package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import java.util.function.Consumer;

import java.lang.invoke.SwitchPoint;
import java.util.logging.Logger;

public class ValidateTrajectoryMirrorLogic {

    enum AllianceColor {
        RED,
        BLUE
    }

    enum FieldStartPosition {
        LEFT,
        RIGHT
    }

    static Pose2d getInitialPose(){
        return new Pose2d(16,-63, Math.toRadians(90));
    }
    static Pose2d getChamberPose(){
        return new Pose2d(8,-31, Math.toRadians(90));
    }
    static Pose2d getChamberTwoPose(){
        return new Pose2d(5,-31, Math.toRadians(90));
    }
    static Pose2d getChamberThreePose(){
        return new Pose2d(2,-31, Math.toRadians(90));
    }
    static Pose2d getFirstSpikeMarkPose(){
        return new Pose2d(35, -35, Math.toRadians(30));
    }
    static Pose2d getSecondSpikeMarkPose(){
        return new Pose2d(45, -35, Math.toRadians(30));
    }
    static Pose2d getObservationZonePose(){
        return new Pose2d(47, -58, Math.toRadians(315));
    }
    static Pose2d getBasketPose(){
        return new Pose2d(48, -48, Math.toRadians(315));
    }
    static Pose2d getParkSubmersiblePose(){
        return new Pose2d(24, -6.5, Math.toRadians(0));
    }

    static Pose2d mirrorPose(double x, double y, double heading, String targetQuadrant) {
        double mirroredX = x;
        double mirroredY = y;
        double mirroredHeading = heading;

        switch (targetQuadrant){
            case "UPPER_RIGHT":
                mirroredY = -y;
                mirroredHeading = -heading;
                break;
            case "UPPER_LEFT":
                mirroredX = -x;
                mirroredY = -y;
                mirroredHeading = Math.PI + heading;
                break;
            case "LOWER_LEFT":
                mirroredX = -x;
                mirroredHeading = Math.PI - heading;
                break;
            case "LOWER_RIGHT":
                break;
            default:
                throw new IllegalArgumentException("Invalid target quadrant: " + targetQuadrant);
        }

        mirroredHeading = (mirroredHeading + 2 * Math.PI) % (2 * Math.PI);
        return new Pose2d(mirroredX, mirroredY, mirroredHeading);
    }
    static double mirrorComponent(double component, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        if (allianceColor == AllianceColor.BLUE || fieldStartPosition == FieldStartPosition.LEFT){
            component += 180;
        }
        return (component % 360 + 360) % 360;
    };

    static Pose2d[] getStartAndEndPose(Pose2d startPose, Pose2d endPose, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        Pose2d[] returnArray = new Pose2d[2];
        String targetQuadrant = "LOWER_RIGHT";

        switch(allianceColor){
            case BLUE:
                if (fieldStartPosition == FieldStartPosition.LEFT){
                    targetQuadrant = "UPPER_RIGHT";
                }
                else {
                    targetQuadrant = "UPPER_LEFT";
                }
                break;
            case RED:
            default:
                if (fieldStartPosition == FieldStartPosition.LEFT){
                    targetQuadrant = "LOWER_LEFT";
                }
                break;

        }
        startPose = mirrorPose(startPose.position.x, startPose.position.y, startPose.heading.toDouble(), targetQuadrant);
        endPose = mirrorPose(endPose.position.x,endPose.position.y, endPose.heading.toDouble(), targetQuadrant);

//        System.out.printf(targetQuadrant);
        returnArray[0] = startPose;
        returnArray[1] = endPose;

        return returnArray;
    }

    static double getEndTangent(double endTangent, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        if (allianceColor == AllianceColor.BLUE){
            endTangent += 180;
        }

        return (endTangent % 360 + 360) % 360;
    }

    static Action getTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition,
                                Pose2d startPose, Pose2d endPose, double tangentAngle, boolean isReversed, String pathOption) {
        DriveShim rrdrive = mybot.getDrive();
        TrajectoryActionBuilder tab;
        if (rrdrive == null) {
            System.out.println("DriveShim is null; check mybot.getDrive() implementation.");
            return null;
        }

        Pose2d[] startAndEndPose = getStartAndEndPose(startPose, endPose, allianceColor, fieldStartPosition);
        Pose2d initialPose = startAndEndPose[0];
        Pose2d finalPose = startAndEndPose[1];

        System.out.println("Initial Pose: " + initialPose);
        System.out.println("Final Pose: " + finalPose);
        switch (pathOption){
            case "TURN":
                tab = rrdrive.actionBuilder(initialPose)
                        .setReversed(isReversed)
                        .turnTo(finalPose.heading);
                break;
            case "LINETOY":
                tab = rrdrive.actionBuilder(initialPose)
                        .setReversed(isReversed)
                        .lineToY(finalPose.position.y);
                break;
            case "SPLINE":
            default:
                tab = rrdrive.actionBuilder(initialPose)
                        .setReversed(isReversed)
                        .splineToLinearHeading(finalPose, Math.toRadians(getEndTangent(tangentAngle, allianceColor, fieldStartPosition)));
                break;

        }


        return tab.build();
    }
    static Action getScoreChamberTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition) {
        return getTrajectory(
                mybot,
                allianceColor,
                fieldStartPosition,
                getInitialPose(),
                getChamberPose(),
                90,
                false,
                "SPLINE"
        );
    }
    static Action getFirstSpikeMarkTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition) {
        return getTrajectory(
                mybot,
                allianceColor,
                fieldStartPosition,
                getChamberPose(),
                getFirstSpikeMarkPose(),
                90,
                true,
                "SPLINE"
        );
    }
    static Action getObservationZoneTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition) {
        return getTrajectory(
                mybot,
                allianceColor,
                fieldStartPosition,
                getFirstSpikeMarkPose(),
                getObservationZonePose(),
                270,
                false,
                "SPLINE"
        );
    }
    static Action getScoreSecondSpecimenTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition) {
        return getTrajectory(
                mybot,
                allianceColor,
                fieldStartPosition,
                getObservationZonePose(),
                getChamberTwoPose(),
                90,
                false,
                "SPLINE"
        );
    }
    // assume start position is always lower right quadrant, aka red observation zone
//    static Action getScoreChamberTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
//        DriveShim rrdrive = mybot.getDrive();
//
//        Pose2d[] startAndEndPose = getStartAndEndPose(new Pose2d(16,-63, Math.toRadians(90)), new Pose2d(8,-31, Math.toRadians(90)), allianceColor, fieldStartPosition);
//        Pose2d initialPose = startAndEndPose[0];
//        Pose2d endPose = startAndEndPose[1];
//
//        TrajectoryActionBuilder tab = rrdrive.actionBuilder(initialPose)
//                .splineToLinearHeading(endPose, Math.toRadians(getEndTangent(90, allianceColor, fieldStartPosition)));
//
//        return tab.build();
//    }

//        static Action getFirstSpikeMarkTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
//        DriveShim rrdrive = mybot.getDrive();
//
//        Pose2d[] startAndEndPose = getStartAndEndPose(new Pose2d(8,-31, Math.toRadians(90)), new Pose2d(35, -35, Math.toRadians(30)), allianceColor, fieldStartPosition);
//        Pose2d initialPose = startAndEndPose[0];
//        Pose2d endPose = startAndEndPose[1];
//
//        TrajectoryActionBuilder tab = rrdrive.actionBuilder(initialPose)
//                .setReversed(true)
//                .splineToLinearHeading(endPose, Math.toRadians(getEndTangent(90, allianceColor, fieldStartPosition)));
//
//        return tab.build();
//    }
//    static Action getObservationZoneTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
//        DriveShim rrdrive = mybot.getDrive();
//
//        Pose2d[] startAndEndPose = getStartAndEndPose(new Pose2d(35, -35, Math.toRadians(30)), new Pose2d(47, -58, Math.toRadians(315)), allianceColor, fieldStartPosition);
//        Pose2d initialPose = startAndEndPose[0];
//        Pose2d endPose = startAndEndPose[1];
//
//        TrajectoryActionBuilder tab = rrdrive.actionBuilder(initialPose)
//                .splineToLinearHeading(endPose, Math.toRadians(getEndTangent(270, allianceColor, fieldStartPosition)));
//
//        return tab.build();
//    }


    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        AllianceColor allianceColor = AllianceColor.BLUE;
        FieldStartPosition fieldStartPosition = FieldStartPosition.LEFT;


//        Action scoreChamberTraj = getScoreChamberTrajectory(myBot, allianceColor, fieldStartPosition);
//        Action firstSpikeMarkTraj = getFirstSpikeMarkTrajectory(myBot, allianceColor, fieldStartPosition);
//        Action observationZoneTraj = getObservationZoneTrajectory(myBot, allianceColor, fieldStartPosition);
//        Action scoreChamberTraj = getTrajectory(myBot,allianceColor,fieldStartPosition,getInitialPose(),getChamberPose(),90,false);
//        Action firstSpikeMarkTraj = getTrajectory(myBot, allianceColor, fieldStartPosition, getChamberPose(),getFirstSpikeMarkPose(), 90, true);
//        Action observationZoneTraj = getTrajectory(myBot, allianceColor,fieldStartPosition,getFirstSpikeMarkPose(), getObservationZonePose(), 270, false);
//        Action secondSpikeMarkTraj = getTrajectory(myBot, allianceColor, fieldStartPosition, getObservationZonePose(), getSecondSpikeMarkPose(), 180, true);
//        Action secondObservationZoneTraj = getTrajectory(myBot)



        if (fieldStartPosition == FieldStartPosition.LEFT){
            myBot.runAction(new SequentialAction(
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getInitialPose(),getChamberPose(),90,false, "SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getChamberPose(),getFirstSpikeMarkPose(), 180, true, "SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getFirstSpikeMarkPose(),getBasketPose(), 180, false, "SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getBasketPose(),getSecondSpikeMarkPose(),180, false, "SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getSecondSpikeMarkPose(),getBasketPose(),180,false,"SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getBasketPose(),getParkSubmersiblePose(),0, true, "SPLINE")
            ));
        }
        else{
            myBot.runAction(new SequentialAction(
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getInitialPose(),getChamberPose(),90,false, "SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getChamberPose(), getFirstSpikeMarkPose(), 90, true, "SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getFirstSpikeMarkPose(),getObservationZonePose(),270, false,"SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getObservationZonePose(),getSecondSpikeMarkPose(),180, true, "SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition,getSecondSpikeMarkPose(),getObservationZonePose(),180, false, "SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition, getObservationZonePose(), new Pose2d(getObservationZonePose().position.x, getObservationZonePose().position.y, Math.toRadians(270)),180,false,"TURN"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition, new Pose2d(getObservationZonePose().position.x, getObservationZonePose().position.y, Math.toRadians(270)), new Pose2d(getObservationZonePose().position.x, getInitialPose().position.y, Math.toRadians(270)), 180, false, "LINETOY"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition, new Pose2d(getObservationZonePose().position.x, getInitialPose().position.y, Math.toRadians(270)), getChamberTwoPose(), 90, true, "SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition, getChamberTwoPose(), getObservationZonePose(),270,true,"SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition, getObservationZonePose(), new Pose2d(getObservationZonePose().position.x, getObservationZonePose().position.y, Math.toRadians(270)),180,false,"TURN"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition, new Pose2d(getObservationZonePose().position.x, getObservationZonePose().position.y, Math.toRadians(270)), new Pose2d(getObservationZonePose().position.x, getInitialPose().position.y, Math.toRadians(270)), 180, false, "LINETOY"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition, new Pose2d(getObservationZonePose().position.x, getInitialPose().position.y, Math.toRadians(270)), getChamberThreePose(), 90, true, "SPLINE"),
                    getTrajectory(myBot,allianceColor,fieldStartPosition, getChamberThreePose(), getObservationZonePose(), 270, true, "SPLINE")
            ));
        }


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}