package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Sweeper;

@TeleOp(name="Nobot: Sweeper Test", group="Nobot")
public class SweeperTest extends LinearOpMode {
    Sweeper sweeper;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        double elbowPos;
        double wheel;
        double elbowPosSet = 0.5;
        double wheelSet = 0.5;
        sweeper = new Sweeper(hardwareMap);
        Servo gripper = hardwareMap.get(Servo.class, "gripper");
        Servo bumper = hardwareMap.get(Servo.class, "bumper");

        waitForStart();

        while (opModeIsActive()) {
            //map joystick range onto servo range [-1.0,1.0] -> [0.0,1.0]
            double desiredWheelSet = (gamepad1.left_stick_x + 1.0) / 2.0;
            double desiredElbowSet = (gamepad1.left_stick_y + 1.0) / 2.0;


            if(gamepad1.y) {
                wheel = 1.0;
            }
            else if(gamepad1.a) {
                wheel = 0.0;
            }
            else if(gamepad1.b || gamepad1.x) {
                wheel = wheelSet;
            }
            else if(gamepad1.right_bumper) {
                wheel = desiredWheelSet;
                wheelSet = wheel;
            }
            else {
                wheel = 0.5;
            }

            if(gamepad1.dpad_up) {
                elbowPos = 1.0;
            }
            else if(gamepad1.dpad_down) {
                elbowPos = 0.0;
            }
            else if(gamepad1.dpad_right || gamepad1.dpad_left) {
                elbowPos = 0.5;
            }
            else if(gamepad1.left_bumper) {
                elbowPos = desiredElbowSet;
                elbowPosSet = elbowPos;
            }
            else {
                elbowPos = elbowPosSet;
            }

            gripper.setPosition(gamepad1.left_trigger);
            bumper.setPosition(gamepad1.right_trigger);

            telemetry.addData("Desired Elbow Set", desiredElbowSet);
            telemetry.addData("Desired Wheel Set", desiredWheelSet);
            telemetry.addData("Commanded Wheel", wheel);
            telemetry.addData("Commanded Elbow", elbowPos);
            sweeper.update(elbowPos, wheel);
            telemetry.addData("Color detected:", sweeper.pattern);
            telemetry.update();
        }

    }

}
