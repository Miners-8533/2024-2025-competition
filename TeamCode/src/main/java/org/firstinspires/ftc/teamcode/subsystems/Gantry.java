package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    public void update() {
        lift.update();
        reach.update();
        sweeper.update();
        colorDetected = sweeper.colorDetected;
    }
    public void setTarget(int liftPos, int reachPos, double elbowPos, double wheelSpd, double gripperPos) {
        lift.setTarget(liftPos);
        reach.setTarget(reachPos);
        sweeper.setTarget(elbowPos, wheelSpd);
        gripper.setPosition(gripperPos);
    }
    public boolean isLiftDone() {
        return lift.isDone();
    }
    public boolean isReachDone() {return reach.isDone();}
    public void log(Telemetry tele) {
        lift.log(tele);
        reach.log(tele);
        sweeper.log(tele);
        tele.addData("Gripper Position: ", gripper.getPosition());
    }

    public void autonLog(TelemetryPacket packet) {
        lift.autonLog(packet);
        reach.autonLog(packet);
    }
}
