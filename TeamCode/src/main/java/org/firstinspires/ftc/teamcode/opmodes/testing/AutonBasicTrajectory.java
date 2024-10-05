package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name="Auton Basic Trajectory", group="Testing")
public class AutonBasicTrajectory extends LinearOpMode {

    @Override
    public void runOpMode() {

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
                chosenTrajectory
        );
    }
}
