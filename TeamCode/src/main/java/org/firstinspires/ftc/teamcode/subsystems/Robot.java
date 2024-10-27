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
    private MotorWithEncoderAndController climber;
    private double climberState;
    private RevBlinkinLedDriver lights;
    private DriveStation driveStation;
    private RobotState robotState;
    private enum RobotState {
        START,
        READY,
        AQUIRE_SAMPlE,
        SCORE_CHAMBER,
        SCORE_HIGH_BASKET
    }

    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController) {
        Pose2d initialPos = new Pose2d(0,0,0);
        chassis = new Chassis(hardwareMap, initialPos);
        driveStation = new DriveStation(driverController, operatorController);
        robotState = RobotState.START;
    }

    public void updateTeleOp(Telemetry telemetry) {
        //get fresh inputs first
        driveStation.update();

        //Put finite state machine here
        switch(robotState) {
            case READY:
                break;
            case START:
                break;
            case AQUIRE_SAMPlE:
                break;
            case SCORE_CHAMBER:
                break;
            case SCORE_HIGH_BASKET:
                break;
            default:
        }

        //climber control
        if(driveStation.isClimb) {
            climberState = SubSystemConfigs.CLIMB_ASCENT_1;
        } else if (driveStation.isClimbPrep) {
            climberState = SubSystemConfigs.CLIMB_PRE_CLIMB;
        } else {
            climberState = SubSystemConfigs.CLIMB_HOME;
        }

        //climber();
        chassis.update(driveStation.forward, driveStation.strafe, driveStation.rotation,true);
    }
}
