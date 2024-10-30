package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.FieldStartPosition;

public class TrajectoryConfig {
    PoseConfig poseConfig = new PoseConfig();
    private MecanumDrive rrdrive;
    public TrajectoryConfig(MecanumDrive autonDrive){
        this.rrdrive = autonDrive;
    }

public static Pose2d mirrorPose(double x, double y, double heading, String targetQuadrant) {
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

    public static Action getTrajectory(MecanumDrive mybot, AllianceColor allianceColor, FieldStartPosition fieldStartPosition,
                                Pose2d startPose, Pose2d endPose, double tangentAngle, boolean isReversed, String pathOption) {

        TrajectoryActionBuilder tab;
        Pose2d[] startAndEndPose = getStartAndEndPose(startPose, endPose, allianceColor, fieldStartPosition);
        Pose2d initialPose = startAndEndPose[0];
        Pose2d finalPose = startAndEndPose[1];

        switch (pathOption){
            case "TURN":
                tab = mybot.actionBuilder(initialPose)
                        .setReversed(isReversed)
                        .turnTo(finalPose.heading);
                break;
            case "LINETOY":
                tab = mybot.actionBuilder(initialPose)
                        .setReversed(isReversed)
                        .lineToY(finalPose.position.y);
                break;
            case "SPLINE":
            default:
                tab = mybot.actionBuilder(initialPose)
                        .setReversed(isReversed)
                        .splineToLinearHeading(finalPose, Math.toRadians(getEndTangent(tangentAngle, allianceColor, fieldStartPosition)));
                break;
        }

        return tab.build();
    }
}
