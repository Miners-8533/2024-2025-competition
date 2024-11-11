package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="TeleOp - Competition", group="Competition")
public class TeleOpCompetition extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2, PoseStorage.poseStorage);

        waitForStart();

        while (opModeIsActive()) {
            robot.updateTeleOp(telemetry);
        }
    }
}
