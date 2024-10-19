package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Gantry {
    private Lift lift;
    private Reach reach;
    private Sweeper sweeper;
    private Servo gripper;

    public Gantry(HardwareMap hardwareMap) {
        lift = new Lift(hardwareMap);
        reach = new Reach(hardwareMap);
        sweeper = new Sweeper(hardwareMap);
        gripper = hardwareMap.get(Servo.class, "gripper");
    }

    public void update() {
//        lift.update();
//        reach.update();
//        sweeper.update();
//        gripper.setPosition();
    }

    public void log(Telemetry tele) {

    }
}
