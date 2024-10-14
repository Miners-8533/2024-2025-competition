package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Chassis {
    private MecanumDrive drive;

    public Chassis(HardwareMap hardwareMap, Pose2d inital_pose) {
        drive = new MecanumDrive(hardwareMap, inital_pose);
    }

    public void update(double forward, double strafe, double rotation) {
        double heading = drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw();
        heading = Math.toRadians(heading);

        Vector2d translation_in_world_coordinates = Rotation2d.exp(heading).times(new Vector2d(forward, strafe));

        //Chassis drive is run independent of robot state
        drive.setDrivePowers(
                new PoseVelocity2d(
                        translation_in_world_coordinates,
                        rotation
                )
        );
    }

}
