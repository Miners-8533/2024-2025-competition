package org.firstinspires.ftc.teamcode.opmodes.testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name="Auton With Action", group="Testing")
public class AutonWithAction extends LinearOpMode {

    @Override
    public void runOpMode() {
        AServo aServo = new AServo(hardwareMap);

        Pose2d initialPose = new Pose2d(-24, -24, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(24)
                .lineToX(-24);

        // Wait for the game to start (driver presses START)

        waitForStart();

        Action chosenTrajectory;
        chosenTrajectory = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                aServo.OpenServo(),
                                chosenTrajectory
                        ),
                        aServo.CloseServo()
                )
        );

        telemetry.update();
    }
}

