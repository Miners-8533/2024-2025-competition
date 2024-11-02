package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.FeedForwardController;

@TeleOp(name="Nobot: Motors Test", group="Nobot")
@Config
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor;
    public static volatile double P = 0.0;
    public static volatile double I = 0.0;
    public static volatile double D = 0.0;
    public static volatile double F = 0.0;
    public static volatile double S = 0.0;
    public static volatile int TARGET_POS = 0;
    private FeedForwardController ffc = new FeedForwardController(new PIDFCoefficients(P,I,D,F));
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //climb, lift, or reach
        motor = hardwareMap.get(DcMotorEx.class, "lift");
        motor.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            if(gamepad1.y) {
                ffc.coefficients = new PIDFCoefficients(P,I,D,F);
                ffc.kStiction = S;
            }
            else if(gamepad1.a) {
                ffc.targetPosition = TARGET_POS;
            }
            else {
                ffc.targetPosition = -100;
            }

            if(gamepad1.right_bumper) {
                motor.setMotorEnable();
                motor.setPower(ffc.update(motor.getCurrentPosition()));
                telemetry.addData("Motor: ", "Enabled");
            } else if(gamepad1.left_bumper) {
                motor.setMotorEnable();
                telemetry.addData("Motor: ", "Enabled");
                motor.setPower(gamepad1.left_stick_y);
            } else {
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
                ffc.targetPosition);
        tele.addData("Climb motor power (+/-%FS)",
                motor.getPower());
        tele.addData("Velocity: ", motor.getVelocity());
        tele.addData("Coef:", ffc.coefficients);
    }
}
