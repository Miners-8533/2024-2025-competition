package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Robot {
    private Sweeper sweeper;
    private Chassis chassis;
    private DriveStation driveStation;

    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController) {
        sweeper = new Sweeper(hardwareMap);
        chassis = new Chassis(hardwareMap, new Pose2d(0,0,0));
        driveStation = new DriveStation(driverController, operatorController);
    }

    public void updateTeleOp(Telemetry telemetry) {
        driveStation.update();
        sweeper.update(0,0);
        chassis.update(driveStation.forward, driveStation.strafe, driveStation.rotation,true);

    }
}
