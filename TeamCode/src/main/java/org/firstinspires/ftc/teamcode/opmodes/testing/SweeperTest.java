package org.firstinspires.ftc.teamcode.opmodes.testing;

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
        double elbowPos;
        double wheel;
        sweeper = new Sweeper(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.a) {
                wheel = 1.0;
                telemetry.addData("Servo", "forward");
            }
            else if(gamepad1.b) {
                wheel = 0.0;
                telemetry.addData("Servo", "backward");
            }
            else if(gamepad1.y) {
                wheel = 0.6;
                telemetry.addData("Servo", "slight forward");
            }
            else {
                wheel = 0.5;
                telemetry.addData("Servo", "stopped");
            }

            if(gamepad2.a) {
                elbowPos = 1.0;
                telemetry.addData("Servo", "forward");
            }
            else if(gamepad2.b) {
                elbowPos = 0.0;
                telemetry.addData("Servo", "backward");
            }
            else if(gamepad2.y) {
                elbowPos = 0.6; // Close to aquasistion
                telemetry.addData("Servo", "slight forward");
            }
            else {
                elbowPos = 0.5; //closer to down
                telemetry.addData("Servo", "stopped");
            }

            sweeper.update(elbowPos, wheel);
            telemetry.update();
        }

    }

}
