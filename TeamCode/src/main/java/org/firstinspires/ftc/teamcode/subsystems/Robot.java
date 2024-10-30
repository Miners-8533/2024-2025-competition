package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private Chassis chassis;
    private Gantry gantry;
    private Servo bumper;
    private MotorWithEncoderAndController climber;
    private RevBlinkinLedDriver lights;
    private DriveStation driveStation;
    private RobotState robotState;
    private int liftScrub = 0;
    private int reachScrub = 0;
    private ElapsedTime darylsTimer = new ElapsedTime();
    private boolean isScoreRetract = false;
    private enum RobotState {
        READY,
        ACQUIRING_SPECIMEN,
        ACQUIRED_SPECIMEN,
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

                isScoreRetract = false;

                if(driveStation.isOutakeSample) {
                    elbowState = SubSystemConfigs.ELBOW_READY_POS;
                    wheelState = SubSystemConfigs.WHEEL_SCORE_SPD;
                } else if(gantry.colorDetected != RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE) {
                    elbowState = SubSystemConfigs.ELBOW_READY_POS;
                    wheelState = SubSystemConfigs.WHEEL_HOLD_SPD;
                } else if(driveStation.isAquireSample) {
                    elbowState = SubSystemConfigs.ELBOW_ACQUIRE_POS;
                    wheelState = SubSystemConfigs.WHEEL_ACQUIRE_SPD;
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
                    darylsTimer.reset();
                    robotState = RobotState.ACQUIRING_SPECIMEN;
                }
                break;
            case ACQUIRING_SPECIMEN:
                newRobotOutputs(
                        SubSystemConfigs.LIFT_HOME_POS,
                        SubSystemConfigs.BUMPER_UP,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.GRIPPER_HOLD_POS,
                        SubSystemConfigs.WHEEL_STOP_SPD
                );
                if (darylsTimer.seconds() > 0.5) {
                    robotState = RobotState.ACQUIRED_SPECIMEN;
                } else if (driveStation.isReady) {
                    robotState = RobotState.READY;
                }
                break;
            case ACQUIRED_SPECIMEN:
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
                        SubSystemConfigs.GRIPPER_CLOSED_POS,
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
                        SubSystemConfigs.LIFT_HIGH_BASKET_POS,
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
        if(robotState == RobotState.READY) {
            reachScrub += SubSystemConfigs.REACH_SCRUB_SPD * driveStation.reachScrub;
            //limit scrub to between max and min targets
            if(gantry.colorDetected == RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE) {
                reachScrub = Math.max(Math.min(reachScrub, SubSystemConfigs.REACH_HOME_POS), SubSystemConfigs.REACH_FULL_EXTEND_POS);
            } else {
                reachScrub = SubSystemConfigs.REACH_HOME_POS;
            }
        } else if(robotState == RobotState.SCORE_HIGH_BASKET) {
            if(driveStation.isOutakeSample && (darylsTimer.seconds() > 0.5)) {
                if(isScoreRetract) {
                    reachScrub = SubSystemConfigs.REACH_HOME_POS;
                    isScoreRetract = false;
                } else {
                    darylsTimer.reset();
                    isScoreRetract = true;
                }
            } else {
                reachScrub += SubSystemConfigs.REACH_SCRUB_SPD * driveStation.reachScrub;
                //limit scrub to between max and min targets
                reachScrub = Math.max(Math.min(reachScrub, SubSystemConfigs.REACH_HOME_POS), SubSystemConfigs.REACH_FULL_EXTEND_POS);
            }
        } else {
            reachScrub = SubSystemConfigs.REACH_HOME_POS;
        }
        robotOutputs.reach = reachScrub;

        telemetry.addData("Daryl's timer ", darylsTimer);
        telemetry.addData("isScoreRetract", isScoreRetract);
        telemetry.addData("isOutakeSample",driveStation.isOutakeSample);

        //TODO WE NEED GYRO RESET
        //TODO Movement limits when lift is up high basket

        //state independent
        if(driveStation.isClimb) {
            robotOutputs.climber = SubSystemConfigs.CLIMB_ASCENT_1;
        } else if (driveStation.isClimbPrep) {
            robotOutputs.climber = SubSystemConfigs.CLIMB_PRE_CLIMB;
        } else if (driveStation.isClimbReset){
            robotOutputs.climber = SubSystemConfigs.CLIMB_HOME;
        }

        //lift scrub
        if(robotState == RobotState.ACQUIRED_SPECIMEN || robotState == RobotState.SCORE_HIGH_BASKET) {
            liftScrub += SubSystemConfigs.LIFT_SCRUB_SPD * driveStation.liftScrub;
            robotOutputs.lift += liftScrub;
        } else {
            liftScrub = 0;
        }

        //bumper down overide
        if(driveStation.isBumperDown) {
            robotOutputs.bumper = SubSystemConfigs.BUMPER_DOWN;
        }

        //all outputs update
        lights.setPattern(gantry.colorDetected);
        bumper.setPosition(robotOutputs.bumper);
        climber.update(robotOutputs.climber);
        gantry.update(robotOutputs.lift,robotOutputs.reach,robotOutputs.elbow,robotOutputs.wheel,robotOutputs.gripper);
        chassis.update(driveStation.forward, driveStation.strafe, driveStation.rotation,true);

        telemetry.addData("State ", robotState);
        telemetry.addData("Reach Scrub", driveStation.reachScrub);

        climber.log(telemetry);
        gantry.log(telemetry);
        telemetry.update();
    }

    private void newRobotOutputs(int liftPos, double bumperPos, double elbowPos, double gripperPos, double wheelSpd) {
        robotOutputs.lift = liftPos;
        robotOutputs.bumper = bumperPos;
        robotOutputs.elbow = elbowPos;
        robotOutputs.gripper = gripperPos;
        robotOutputs.wheel = wheelSpd;
    }
    public Action autonStart(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.update(
                        SubSystemConfigs.LIFT_HIGH_CHAMBER_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.WHEEL_STOP_SPD,
                        SubSystemConfigs.GRIPPER_CLOSED_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_DOWN);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return gantry.isLiftDone();
            }
        };
    }
    public Action scoreSpecimen(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.update(
                        SubSystemConfigs.LIFT_HIGH_CHAMBER_SCORE_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.WHEEL_STOP_SPD,
                        SubSystemConfigs.GRIPPER_CLOSED_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_DOWN);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return gantry.isLiftDone();
            }
        };
    }
    public Action goToReadyPose(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.update(
                        SubSystemConfigs.LIFT_HOME_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.WHEEL_STOP_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return gantry.isLiftDone();
            }
        };
    }
    public Action floorAcquire(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.update(
                        SubSystemConfigs.LIFT_HOME_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_ACQUIRE_POS,
                        SubSystemConfigs.WHEEL_ACQUIRE_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return (gantry.colorDetected != RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
            }
        };
    }
    public Action prepareScoreHighBasket(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.update(
                        SubSystemConfigs.LIFT_HIGH_CHAMBER_SCORE_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_SCORE_BASKET_POS,
                        SubSystemConfigs.WHEEL_HOLD_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return gantry.isLiftDone();
            }
        };
    }
    public Action scoreHighBasket(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.update(
                        SubSystemConfigs.LIFT_HIGH_CHAMBER_SCORE_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_SCORE_BASKET_POS,
                        SubSystemConfigs.WHEEL_SCORE_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return true; //need to use external timer
            }
        };
    }
    public Action outakeSampleGround(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.update(
                        SubSystemConfigs.LIFT_HOME_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_SCORE_BASKET_POS,
                        SubSystemConfigs.WHEEL_SCORE_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return true; //need to use external timer
            }
        };
    }
    public Action acquireSpecimen(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.update(
                        SubSystemConfigs.LIFT_HOME_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.WHEEL_STOP_SPD,
                        SubSystemConfigs.GRIPPER_CLOSED_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return true; //need to use external timer
            }
        };
    }
}
