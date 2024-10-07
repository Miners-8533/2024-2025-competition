package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@TeleOp(name="Continuous Servo Rotation", group="Testing")
public class ConceptContinuousServo extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo continousServo = hardwareMap.get(Servo.class, "continuous_servo");

        continousServo.setPosition(0.5); // move to neutral position on init

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                continousServo.setPosition(1.0);
                telemetry.addData("Servo", "forward");
            }
            else if(gamepad1.b) {
                continousServo.setPosition(0.0);
                telemetry.addData("Servo", "backward");
            }
            else {
                continousServo.setPosition(0.5);
                telemetry.addData("Servo", "stopped");
            }

            telemetry.update();
        }
    }
}

