package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ValidateTrajectoryMirrorLogic {
    enum AllianceColor {
        RED,
        BLUE
    }

    enum FieldStartPosition {
        LEFT,
        RIGHT
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

    static double mirrorAxis(double component){
        return -component;
    }

    static Action ScoreChamberTrajectory(RoadRunnerBotEntity mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        // assume start position is always lower right quadrant, aka red observation zone
        DriveShim rrdrive = mybot.getDrive();
        double endTangent = 0;
        Pose2d initialPose = new Pose2d(16,-63, Math.toRadians(90));
        Pose2d chamberPose = new Pose2d(8,-31, Math.toRadians(90));

        if (fieldStartPosition == FieldStartPosition.LEFT){
            initialPose = new Pose2d(mirrorAxis(initialPose.position.x), initialPose.position.y, initialPose.heading.toDouble());
            chamberPose = new Pose2d(mirrorAxis(chamberPose.position.x), chamberPose.position.y, chamberPose.heading.toDouble());
        }

        if (allianceColor == AllianceColor.BLUE){
            endTangent = 180;
            initialPose = mirrorPose(initialPose.position.x, initialPose.position.y, initialPose.heading.toDouble());
            chamberPose = mirrorPose(chamberPose.position.x, chamberPose.position.y, chamberPose.heading.toDouble());
        }

        TrajectoryActionBuilder tab1 = rrdrive.actionBuilder(initialPose)
                .splineToLinearHeading(chamberPose, Math.toRadians(90 + endTangent));

        return tab1.build();
    }


    public static void main(String[] args) {


        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        AllianceColor allianceColor = AllianceColor.BLUE;
        FieldStartPosition fieldStartPosition = FieldStartPosition.RIGHT;


        Pose2d initialPose = new Pose2d(-16,-63, Math.toRadians(90));

        Pose2d scoreChamber = new Pose2d(-8,-31, Math.toRadians(90));

        Pose2d scoreHighBasket = new Pose2d(-48, -48, Math.toRadians(225));

        Pose2d firstSpikeMark = new Pose2d(-35, -35, Math.toRadians(150));

        Pose2d secondSpikeMark = new Pose2d(-45, -35, Math.toRadians(150));

        Pose2d parkNearSubmersible = new Pose2d(-24, -6.5, Math.toRadians(180));

        Action validateMirrorLogic = ScoreChamberTrajectory(myBot, allianceColor, fieldStartPosition);

        myBot.runAction(validateMirrorLogic);

//        myBot.runAction(myBot.getDrive().actionBuilder(initialPose)
//                .splineToLinearHeading(scoreChamber, Math.toRadians(90))
//                .waitSeconds(2)
//                .setReversed(true)
//                .splineToLinearHeading(firstSpikeMark, Math.toRadians(180))
//                .waitSeconds(2)
//                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180))
//                .waitSeconds(2)
//                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180))
//                .waitSeconds(2)
//                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180))
//                .waitSeconds(2)
//                .splineToLinearHeading(parkNearSubmersible, Math.toRadians(0))
//                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}