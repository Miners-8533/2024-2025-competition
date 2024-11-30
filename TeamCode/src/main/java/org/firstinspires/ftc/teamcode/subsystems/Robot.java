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
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

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
    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController, Pose2d initialPose) {
//        Pose2d initialPos = new Pose2d(0,0,Math.toRadians(90));
        chassis = new Chassis(hardwareMap, initialPose);
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
                } else if (driveStation.isTargetSample) {
                    elbowState = SubSystemConfigs.ELBOW_TARGET_POS;
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
                        SubSystemConfigs.WHEEL_SCORE_SPD
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
                } else if (driveStation.isAquireSpecimen){
                    robotState = RobotState.ACQUIRED_SPECIMEN;
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

        //set all targets
        lights.setPattern(gantry.colorDetected);
        bumper.setPosition(robotOutputs.bumper);
        climber.setTarget(robotOutputs.climber);
        gantry.setTarget(robotOutputs.lift,robotOutputs.reach,robotOutputs.elbow,robotOutputs.wheel,robotOutputs.gripper);

        //all outputs update
        climber.update();
        gantry.update();
        chassis.update(driveStation.forward, driveStation.strafe, driveStation.rotation,true, driveStation.isGyroReset);

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
                gantry.setTarget(
                        SubSystemConfigs.LIFT_HIGH_CHAMBER_AUTON_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.WHEEL_STOP_SPD,
                        SubSystemConfigs.GRIPPER_CLOSED_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_DOWN);
                return !gantry.isLiftDone();
            }
        };
    }
    public Action scoreSpecimen(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.setTarget(
                        SubSystemConfigs.LIFT_HIGH_CHAMBER_SCORE_AUTO_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.WHEEL_STOP_SPD,
                        SubSystemConfigs.GRIPPER_CLOSED_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                return !gantry.isLiftDone(); //|| !gantry.isReachDone());
            }
        };
    }
    public Action goToReadyPose(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.setTarget(
                        SubSystemConfigs.LIFT_HOME_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.WHEEL_STOP_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return false;//(!gantry.isLiftDone() || !gantry.isReachDone());
            }
        };
    }
    public Action floorAcquire(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.setTarget(
                        SubSystemConfigs.LIFT_HOME_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_ACQUIRE_POS,
                        SubSystemConfigs.WHEEL_ACQUIRE_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return false;//(!gantry.isLiftDone() || !gantry.isReachDone());
            }
        };
    }
    public Action floorAcquireReach(){
        return new Action(){
            private boolean intialized = false;
            private int tempReachVal;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!intialized){
                    tempReachVal = 0;
                    intialized = true;
                }
                gantry.setTarget(
                        SubSystemConfigs.LIFT_HOME_POS,
                        tempReachVal,
                        SubSystemConfigs.ELBOW_ACQUIRE_POS,
                        SubSystemConfigs.WHEEL_ACQUIRE_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                tempReachVal -= SubSystemConfigs.REACH_SCRUB_SPD_AUTON;
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                if (tempReachVal >= SubSystemConfigs.REACH_FLOOR_EXTEND_POS){
                    return true;
                } else{
                    return false;
                }
            }
        };
    }
    public Action prepareScoreHighBasket(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.setTarget(
                        SubSystemConfigs.LIFT_HIGH_BASKET_AUTON_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_SCORE_BASKET_POS,
                        SubSystemConfigs.WHEEL_HOLD_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return (!gantry.isLiftDone());
            }
        };
    }
    public Action scoreHighBasket(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.setTarget(
                        SubSystemConfigs.LIFT_HIGH_BASKET_AUTON_POS,
                        SubSystemConfigs.REACH_HIGH_BASKET_EXTEND_POS,
                        SubSystemConfigs.ELBOW_SCORE_BASKET_POS,
                        SubSystemConfigs.WHEEL_SCORE_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return (!gantry.isLiftDone() || !gantry.isReachDone()); //need to use external timer
            }
        };
    }
    public Action highBasketReach(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.setTarget(
                        SubSystemConfigs.LIFT_HIGH_BASKET_AUTON_POS,
                        SubSystemConfigs.REACH_HIGH_BASKET_EXTEND_POS,
                        SubSystemConfigs.ELBOW_SCORE_BASKET_POS,
                        SubSystemConfigs.WHEEL_HOLD_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return (!gantry.isLiftDone() || !gantry.isReachDone()); //need to use external timer
            }
        };
    }
    public Action outakeSampleGround(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.setTarget(
                        SubSystemConfigs.LIFT_HOME_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_SCORE_BASKET_POS,
                        SubSystemConfigs.WHEEL_SCORE_SPD,
                        SubSystemConfigs.GRIPPER_OPEN_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return (!gantry.isLiftDone() || !gantry.isReachDone()); //need to use external timer
            }
        };
    }
    public Action acquireSpecimen(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                gantry.setTarget(
                        SubSystemConfigs.LIFT_HOME_POS,
                        SubSystemConfigs.REACH_HOME_POS,
                        SubSystemConfigs.ELBOW_READY_POS,
                        SubSystemConfigs.WHEEL_STOP_SPD,
                        SubSystemConfigs.GRIPPER_CLOSED_POS
                );
                bumper.setPosition(SubSystemConfigs.BUMPER_UP);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return !gantry.isLiftDone(); //|| !gantry.isReachDone());//need to use external timer
            }
        };
    }
    public Action setLights(RevBlinkinLedDriver.BlinkinPattern blinkinPattern){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                lights.setPattern(blinkinPattern);
                //maybe use lights?
                //use log functions? or packet.put("Current Lift Position", pos);
                return false;
            }
        };
    }
    public Action setLastPose(MecanumDrive drive){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                PoseStorage.poseStorage = drive.pose;
                return false;
            }
        };
    }
    public Action autonUpdate(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                climber.update();
                gantry.update();
                gantry.autonLog(packet);
                chassis.setPose();
                return true; //never stop
            }
        };
    }
}
