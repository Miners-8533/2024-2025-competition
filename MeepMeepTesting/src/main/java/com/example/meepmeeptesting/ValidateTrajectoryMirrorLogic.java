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
    public Pose2d getChamberTwoPose(){
        return new Pose2d(5,-31, Math.toRadians(90));
    }
    public Pose2d getChamberThreePose(){
        return new Pose2d(2,-31, Math.toRadians(90));
    }
    public Pose2d getFirstSpikeMarkPose(){
        return new Pose2d(35, -35, Math.toRadians(30));
    }
    public Pose2d getSecondSpikeMarkPose(){
        return new Pose2d(45, -35, Math.toRadians(30));
    }
    public Pose2d getObservationZonePose(){
        return new Pose2d(47, -58, Math.toRadians(315));
    }
    public Pose2d getBasketPose(){
        return new Pose2d(-48, -48, Math.toRadians(225));
    }
    public Pose2d getParkSubmersiblePose(){
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
//        System.out.printf(String.valueOf(mirroredX), String.valueOf(mirroredY), String.valueOf(mirroredHeading));
        return new Pose2d(mirroredX, mirroredY, mirroredHeading);
    }

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
        endTangent = (endTangent % 360 + 360) % 360;
//        System.out.printf(String.valueOf(endTangent));

        return endTangent;
    }

//    static Action getTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition,
//                                Pose2d startPose, Consumer<TrajectoryActionBuilder> trajectoryConfigurer) {
//        DriveShim rrdrive = mybot.getDrive();
//        if (rrdrive == null) {
//            System.out.println("DriveShim is null; check mybot.getDrive() implementation.");
//            return null;
//        }
//
//        Pose2d[] startAndEndPose = getStartAndEndPose(startPose, startPose, allianceColor, fieldStartPosition);  // Use startPose array for both if needed
//        Pose2d initialPose = startAndEndPose[0];
//
//        TrajectoryActionBuilder tab = rrdrive.actionBuilder(initialPose);
//
//        // Apply the custom trajectory configurations using lambda
//        trajectoryConfigurer.accept(tab);
//
//        Action action = tab.build();
//        if (action == null) {
//            System.out.println("Action is null; check TrajectoryActionBuilder configuration.");
//        } else {
//            System.out.println("Action successfully created.");
//        }
//
//        return action;
//    }


    // assume start position is always lower right quadrant, aka red observation zone
    static Action getScoreChamberTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        DriveShim rrdrive = mybot.getDrive();

        Pose2d[] startAndEndPose = getStartAndEndPose(new Pose2d(16,-63, Math.toRadians(90)), new Pose2d(8,-31, Math.toRadians(90)), allianceColor, fieldStartPosition);
        Pose2d initialPose = startAndEndPose[0];
        Pose2d endPose = startAndEndPose[1];

        TrajectoryActionBuilder tab = rrdrive.actionBuilder(initialPose)
                .splineToLinearHeading(endPose, Math.toRadians(getEndTangent(90, allianceColor, fieldStartPosition)));

        return tab.build();
    }
//    static Action getScoreChamberTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition) {
//        return getTrajectory(mybot, allianceColor, fieldStartPosition,
//                getInitialPose(),
//                tab -> tab.splineToLinearHeading(getChamberPose(),
//                        Math.toRadians(getEndTangent(90, allianceColor, fieldStartPosition))));
//    }

        static Action getFirstSpikeMarkTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        DriveShim rrdrive = mybot.getDrive();

        Pose2d[] startAndEndPose = getStartAndEndPose(new Pose2d(8,-31, Math.toRadians(90)), new Pose2d(35, -35, Math.toRadians(30)), allianceColor, fieldStartPosition);
        Pose2d initialPose = startAndEndPose[0];
        Pose2d endPose = startAndEndPose[1];

        TrajectoryActionBuilder tab = rrdrive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(endPose, Math.toRadians(getEndTangent(90, allianceColor, fieldStartPosition)));

        return tab.build();
    }
    static Action getObservationZoneTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        DriveShim rrdrive = mybot.getDrive();

        Pose2d[] startAndEndPose = getStartAndEndPose(new Pose2d(35, -35, Math.toRadians(30)), new Pose2d(47, -58, Math.toRadians(315)), allianceColor, fieldStartPosition);
        Pose2d initialPose = startAndEndPose[0];
        Pose2d endPose = startAndEndPose[1];

        TrajectoryActionBuilder tab = rrdrive.actionBuilder(initialPose)
                .splineToLinearHeading(endPose, Math.toRadians(getEndTangent(270, allianceColor, fieldStartPosition)));

        return tab.build();
    }


    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        AllianceColor allianceColor = AllianceColor.RED;
        FieldStartPosition fieldStartPosition = FieldStartPosition.RIGHT;


        Action scoreChamberTraj = getScoreChamberTrajectory(myBot, allianceColor, fieldStartPosition);
        Action firstSpikeMarkTraj = getFirstSpikeMarkTrajectory(myBot, allianceColor, fieldStartPosition);
        Action observationZoneTraj = getObservationZoneTrajectory(myBot, allianceColor, fieldStartPosition);

        if (fieldStartPosition == FieldStartPosition.LEFT){
            myBot.runAction(new SequentialAction(
                    scoreChamberTraj,
                    firstSpikeMarkTraj
            ));
        }
        else{
            myBot.runAction(new SequentialAction(
                    scoreChamberTraj,
                    firstSpikeMarkTraj,
                    observationZoneTraj
            ));
        }


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}