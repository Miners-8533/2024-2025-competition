package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gantry {
    private MotorWithEncoderAndController lift;
    private MotorWithEncoderAndController reach;
    private Sweeper sweeper;
    private Servo gripper;
    public Gantry(HardwareMap hardwareMap) {
        lift = new MotorWithEncoderAndController(hardwareMap,SubSystemConfigs.liftConfig);
        reach = new MotorWithEncoderAndController(hardwareMap,SubSystemConfigs.reachConfig);
        sweeper = new Sweeper(hardwareMap);
        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper.setDirection(Servo.Direction.FORWARD);
    }
    public void update(SubSystemConfigs.GantryState gantryState, double wheelSpd, double gripperPos) {
        lift.update(gantryState.liftPos);
        reach.update(gantryState.reachPos);
        sweeper.update(gantryState.elbowPos, wheelSpd);
        gripper.setPosition(gripperPos);
    }
    public void log(Telemetry tele) {
        lift.log(tele);
        reach.log(tele);
        sweeper.log(tele);
        tele.addData("Gripper Position: ", gripper.getPosition());
    }
}
