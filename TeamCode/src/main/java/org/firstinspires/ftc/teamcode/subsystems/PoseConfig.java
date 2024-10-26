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

    public Pose2d getInitialPose(){
        return new Pose2d(16,-63, Math.toRadians(90));
    }

    public Pose2d getChamberPose(){
        return new Pose2d(8,-31, Math.toRadians(90));
    }

    public Pose2d getFirstSpikeMarkPose(){
        return new Pose2d(35, -35, Math.toRadians(30));
    }

    public Pose2d getSecondSpikeMarkPose(){
        return new Pose2d(45, -35, Math.toRadians(30));
    }
}
