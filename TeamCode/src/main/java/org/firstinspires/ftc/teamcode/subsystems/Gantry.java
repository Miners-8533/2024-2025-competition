package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gantry {
    private MotorWithEncoderAndController lift;
    private MotorWithEncoderAndController reach;
    private Sweeper sweeper;
    private Servo gripper;
    public RevBlinkinLedDriver.BlinkinPattern colorDetected;
    public Gantry(HardwareMap hardwareMap) {
        lift = new MotorWithEncoderAndController(hardwareMap,SubSystemConfigs.liftConfig);
        reach = new MotorWithEncoderAndController(hardwareMap,SubSystemConfigs.reachConfig);
        sweeper = new Sweeper(hardwareMap);
        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper.setDirection(Servo.Direction.FORWARD);
        colorDetected = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
    }
    public void update(int liftPos, int reachPos, double elbowPos, double wheelSpd, double gripperPos) {
        lift.update(liftPos);
        reach.update(reachPos);
        sweeper.update(elbowPos, wheelSpd);
        colorDetected = sweeper.colorDetected;
        gripper.setPosition(gripperPos);
    }
    public void log(Telemetry tele) {
        lift.log(tele);
        reach.log(tele);
        sweeper.log(tele);
        tele.addData("Gripper Position: ", gripper.getPosition());
    }
}
