package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private Chassis chassis;
    private Gantry gantry;
    private Servo bumper;
    private MotorWithEncoderAndController climber;
    private RevBlinkinLedDriver lights;
    private DriveStation driveStation;
    private RobotState robotState;
    private enum RobotState {
        READY,
        AQUIRED_SPECIMEN,
        SCORE_CHAMBER,
        SCORE_HIGH_BASKET
    }
    private class RobotOutputs {
        public int climber;
        public int lift;
        public int reach;
        public double elbow;
        public double gripper;
        public double wheel;
        public double bumper;
    }
    private RobotOutputs robotOutputs;
    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController) {
        Pose2d initialPos = new Pose2d(0,0,0);
        chassis = new Chassis(hardwareMap, initialPos);
        driveStation = new DriveStation(driverController, operatorController);
        bumper = hardwareMap.get(Servo.class, "bumper");
        gantry = new Gantry(hardwareMap);
        climber = new MotorWithEncoderAndController(hardwareMap,SubSystemConfigs.climbConfig);
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        robotState = RobotState.READY;
        robotOutputs = new RobotOutputs();
        //may not need to set initial states
    }
    public void updateTeleOp(Telemetry telemetry) {
        //get fresh inputs first
        driveStation.update();

        //Put finite state machine here
        switch(robotState) {
            case READY:
                double elbowState;
                double wheelState;

                if(gantry.colorDetected != RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE) {
                    elbowState = SubSystemConfigs.ELBOW_READY_POS;
                    wheelState = SubSystemConfigs.WHEEL_HOLD_SPD;
                } else if(driveStation.isAquireSample) {
                    elbowState = SubSystemConfigs.ELBOW_AQUIRE_POS;
                    wheelState = SubSystemConfigs.WHEEL_AQUIRE_SPD;
                } else {
                    elbowState = SubSystemConfigs.ELBOW_READY_POS;
                    wheelState = SubSystemConfigs.WHEEL_STOP_SPD;
                }

                newRobotOutputs(
                        SubSystemConfigs.LIFT_HOME_POS,
                        SubSystemConfigs.BUMPER_UP,
                        elbowState,
                        SubSystemConfigs.GRIPPER_OPEN_POS,
                        wheelState
                        );

                if(driveStation.isScoreBasket) {
                    robotState = RobotState.SCORE_HIGH_BASKET;
                } else if (driveStation.isAquireSpecimen) {
                    robotState = RobotState.AQUIRED_SPECIMEN; //TODO test is to quick?
                }
                break;

            case AQUIRED_SPECIMEN:
                newRobotOutputs(
                        SubSystemConfigs.LIFT_HIGH_CHAMBER_POS,
                        SubSystemConfigs.BUMPER_DOWN,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.GRIPPER_HOLD_POS,
                        SubSystemConfigs.WHEEL_STOP_SPD
                );

                if(driveStation.isScoreSpecimen) {
                    robotState = RobotState.SCORE_CHAMBER;
                } else if (driveStation.isReady) {
                    robotState = RobotState.READY;
                }
                break;
            case SCORE_CHAMBER:
                newRobotOutputs(
                        SubSystemConfigs.LIFT_HIGH_CHAMBER_SCORE_POS,
                        SubSystemConfigs.BUMPER_DOWN,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.GRIPPER_OPEN_POS,//TODO test?
                        SubSystemConfigs.WHEEL_STOP_SPD
                );

                if(driveStation.isReady) {
                    robotState = RobotState.READY;
                }
                break;
            case SCORE_HIGH_BASKET:
                if(driveStation.isOutakeSample) {
                    wheelState = SubSystemConfigs.WHEEL_SCORE_SPD;
                } else {
                    wheelState = SubSystemConfigs.WHEEL_HOLD_SPD;
                }

                newRobotOutputs(
                        SubSystemConfigs.LIFT_HIGH_CHAMBER_SCORE_POS,
                        SubSystemConfigs.BUMPER_UP,
                        SubSystemConfigs.ELBOW_SCORE_BASKET_POS,
                        SubSystemConfigs.GRIPPER_OPEN_POS,
                        wheelState
                );

                if(driveStation.isReady) {
                    robotState = RobotState.READY;
                }
        }

        //reach manual control
        if(robotState == RobotState.READY || robotState == RobotState.SCORE_HIGH_BASKET) {
            robotOutputs.reach = (int)(SubSystemConfigs.REACH_FULL_EXTEND_POS * driveStation.reachScrub);
        } else {
            robotOutputs.reach = SubSystemConfigs.REACH_HOME_POS;
        }

        //state independent
        if(driveStation.isClimb) {
            robotOutputs.climber = SubSystemConfigs.CLIMB_ASCENT_1;
        } else if (driveStation.isClimbPrep) {
            robotOutputs.climber = SubSystemConfigs.CLIMB_PRE_CLIMB;
        } else if (driveStation.isClimbReset){
            robotOutputs.climber = SubSystemConfigs.CLIMB_HOME;
        }

        //all outputs update
        lights.setPattern(gantry.colorDetected);
        bumper.setPosition(robotOutputs.bumper);
        climber.update(robotOutputs.climber);
        gantry.update(robotOutputs.lift,robotOutputs.reach,robotOutputs.elbow,robotOutputs.wheel,robotOutputs.gripper);
        chassis.update(driveStation.forward, driveStation.strafe, driveStation.rotation,true);
    }

    private void newRobotOutputs(int liftPos, double bumperPos, double elbowPos, double gripperPos, double wheelSpd) {
        robotOutputs.lift = liftPos;
        robotOutputs.bumper = bumperPos;
        robotOutputs.elbow = elbowPos;
        robotOutputs.gripper = gripperPos;
        robotOutputs.wheel = wheelSpd;
    }
}
