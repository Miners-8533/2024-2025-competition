package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.FeedForwardController;
import org.firstinspires.ftc.teamcode.subsystems.SubSystemConfigs;

@TeleOp(name="Nobot: Motors Test", group="Nobot")
@Config
public class WingTest extends LinearOpMode {
    private Servo wing;
    public static volatile double TARGET_POS = 1.0;
    public static volatile double HOME_POS = 0.0;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        wing = hardwareMap.get(Servo.class, "wing");

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.a) {
                wing.setPosition(TARGET_POS);
            }
            else {
                wing.setPosition(HOME_POS);
            }

            log(telemetry);
            telemetry.update();
        }

    }
    private void log(Telemetry tele) {
        tele.addData("Wing position",
                wing.getPosition());
    }
}
