package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.FieldStartPosition;

public class PoseConfig {
    private MecanumDrive rrdrive;
    public PoseConfig(MecanumDrive autonDrive){
        this.rrdrive = autonDrive;
    }

    public Pose2d getInitialPose(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        double poseX;
        double poseY;
        double poseH;

        Pose2d whateverPose = new Pose2d(0, 0, 0);

        if (fieldStartPosition == FieldStartPosition.LEFT){
            whateverPose = new Pose2d(whateverPose.position.x, -whateverPose.position.y, Math.toRadians(0));
            poseX = (allianceColor == AllianceColor.RED) ? -16 : 16;
        }
        else {
            poseX = (allianceColor == AllianceColor.RED) ? 16 : -16;
        }

        poseY = (allianceColor == AllianceColor.RED) ? -63 : 63;
        poseH = (allianceColor == AllianceColor.RED) ? 90 : 270;

        return new Pose2d(poseX, poseY, Math.toRadians(poseH));
    }

    public Pose2d getChamberPose(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
        double poseX;
        double poseY;
        double poseH;

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

    public Pose2d getFirstSpikeMarkPose(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
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

    public Pose2d getSecondSpikeMarkPose(AllianceColor allianceColor, FieldStartPosition fieldStartPosition){
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
}
