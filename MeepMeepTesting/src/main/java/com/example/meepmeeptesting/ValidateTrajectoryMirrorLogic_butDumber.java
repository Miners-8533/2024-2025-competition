package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ValidateTrajectoryMirrorLogic_butDumber {

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
        return new Pose2d(-48, -48, Math.toRadians(225));
    }
    static Pose2d getParkSubmersiblePose(){
        return new Pose2d(-24, -6.5, Math.toRadians(180));
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

//    static Pose2d[] getStartAndEndPose(Pose2d startPose, Pose2d endPose, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
//        Pose2d[] returnArray = new Pose2d[2];
//        String targetQuadrant = "LOWER_RIGHT";
//
//        switch(allianceColor){
//            case BLUE:
//                if (fieldStartPosition == FieldStartPosition.LEFT){
//                    targetQuadrant = "UPPER_RIGHT";
//                }
//                else {
//                    targetQuadrant = "UPPER_LEFT";
//                }
//                break;
//            case RED:
//            default:
//                if (fieldStartPosition == FieldStartPosition.LEFT){
//                    targetQuadrant = "LOWER_LEFT";
//                }
//                break;
//
//        }
//        startPose = mirrorPose(startPose.position.x, startPose.position.y, startPose.heading.toDouble(), targetQuadrant);
//        endPose = mirrorPose(endPose.position.x,endPose.position.y, endPose.heading.toDouble(), targetQuadrant);
//
//        returnArray[0] = startPose;
//        returnArray[1] = endPose;
//
//        return returnArray;
//    }



    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        AllianceColor allianceColor = AllianceColor.BLUE;
        FieldStartPosition fieldStartPosition = FieldStartPosition.RIGHT;

//        switch(fieldStartPosition){
//            case LEFT:
//                if (allianceColor == AllianceColor.BLUE){
//                    myBot.runAction(new SequentialAction(
//                            .splineToLinearHeading(getChamberPose(), Math.toRadians(90))
//                            .setReversed(true)
//                            .splineToLinearHeading(getFirstSpikeMarkPose(), Math.toRadians(180))
//                            .waitSeconds(2)
//                            .splineToLinearHeading(scoreHighBasket, Math.toRadians(180))
//                            .waitSeconds(2)
//                            .splineToLinearHeading(secondSpikeMark,Math.toRadians(180))
//                            .waitSeconds(2)
//                            .splineToLinearHeading(scoreHighBasket, Math.toRadians(180))
//                            .waitSeconds(2)
//                            .splineToLinearHeading(parkNearSubmersible, Math.toRadians(0))
//                            .build());
//                    ));
//                }
//        }
//
//
//        if (fieldStartPosition == FieldStartPosition.LEFT){
//            myBot.runAction(new SequentialAction(
//                    getTrajectory(myBot,allianceColor,fieldStartPosition,getInitialPose(),getChamberPose(),90,false),
//                    getTrajectory(myBot,allianceColor,fieldStartPosition,getChamberPose(), getFirstSpikeMarkPose(), 90, true)
//            ));
//        }
//        else{
//            myBot.runAction(new SequentialAction(
//                    getTrajectory(myBot,allianceColor,fieldStartPosition,getInitialPose(),getChamberPose(),90,false),
//                    getTrajectory(myBot,allianceColor,fieldStartPosition,getChamberPose(), getFirstSpikeMarkPose(), 90, true),
//                    getTrajectory(myBot,allianceColor,fieldStartPosition,getFirstSpikeMarkPose(),getObservationZonePose(),270, false),
//                    getTrajectory(myBot,allianceColor,fieldStartPosition,getObservationZonePose(),getSecondSpikeMarkPose(),180, true),
//                    getTrajectory(myBot,allianceColor,fieldStartPosition,getSecondSpikeMarkPose(),getObservationZonePose(),180, false),
//                    turn.build()
//
//
//            )
//
//            );
//        }


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}