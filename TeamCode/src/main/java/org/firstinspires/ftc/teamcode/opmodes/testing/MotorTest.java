package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Nobot: Motors Test", group="Nobot")
@Config
public class MotorTest extends LinearOpMode {

    private DcMotorEx motor;
    public static volatile double P = 0.0;
    public static volatile double I = 0.0;
    public static volatile double D = 0.0;
    public static volatile double F = 0.0;
    public static volatile int TARGET_POS = 0;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //climb, lift, or reach
        motor = hardwareMap.get(DcMotorEx.class, "lift");

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.y) {
                PIDFCoefficients coef = new PIDFCoefficients(P,I,D,F);
                motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, coef);
            }
            else if(gamepad1.a) {
                motor.setTargetPosition(TARGET_POS);
            }
            else {
                motor.setTargetPosition(0);
            }

            if(gamepad1.right_bumper) {
                motor.setMotorEnable();
                telemetry.addData("Motor: ", "Enabled");
            }
            else {
                motor.setMotorDisable();
                telemetry.addData("Motor: ", "Disabled");
            }

            log(telemetry);
            telemetry.update();
        }

    }

    private void log(Telemetry tele) {
        tele.addData("Climb current encoder ticks",
                motor.getCurrentPosition());
        tele.addData("Climb motor current (A)",
                motor.getCurrent(CurrentUnit.AMPS));
        tele.addData("Climb current target position",
                motor.getTargetPosition());
        tele.addData("Climb motor power (+/-%FS)",
                motor.getPower());
    }
}
