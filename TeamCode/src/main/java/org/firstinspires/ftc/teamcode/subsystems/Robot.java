package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Robot {
    private Chassis chassis;
    private Gantry gantry;
    private Climber climber;
    private RevBlinkinLedDriver lights;
    private DriveStation driveStation;

    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController) {
        chassis = new Chassis(hardwareMap, new Pose2d(0,0,0));
        driveStation = new DriveStation(driverController, operatorController);
    }

    public void updateTeleOp(Telemetry telemetry) {
        driveStation.update();
        chassis.update(driveStation.forward, driveStation.strafe, driveStation.rotation,true);
    }
}
