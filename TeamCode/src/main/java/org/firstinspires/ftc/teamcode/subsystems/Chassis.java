package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Chassis {
    private MecanumDrive drive;
    private double gyroOffset;

    public Chassis(HardwareMap hardwareMap, Pose2d initialPose) {
        drive = new MecanumDrive(hardwareMap, initialPose);
        gyroOffset = 0;
    }

    public void update(double forward, double strafe, double rotation, boolean isFieldOrientedControl, boolean isGyroReset) {
        drive.updatePoseEstimate();
        Vector2d commanded_translation;
        double heading = drive.pose.heading.toDouble();
        if (isGyroReset){
            gyroOffset = heading;
        }
        heading = heading - gyroOffset;

        if(isFieldOrientedControl) {
            commanded_translation = rotate(forward, strafe, heading);
        } else {
            commanded_translation = new Vector2d(forward, strafe);
        }

        //Chassis drive is run independent of robot state
        drive.setDrivePowers(
                new PoseVelocity2d(
                        commanded_translation,
                        rotation
                )
        );
    }

    private Vector2d rotate(double x, double y, double theta) {
        return new Vector2d(x * Math.cos(theta) + y * Math.sin(theta),
                -x * Math.sin(theta) + y * Math.cos(theta));
    }

}
