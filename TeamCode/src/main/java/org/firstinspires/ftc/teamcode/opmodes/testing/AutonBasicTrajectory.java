package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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

//        Pose2d initialPose = new Pose2d(-24, -24, 0);
        Pose2d initialPose = new Pose2d(-24,-63, Math.toRadians(90));

        Pose2d scoreChamber = new Pose2d(-8,-31, Math.toRadians(90));

        Pose2d scoreRedBasketLeft = new Pose2d(-41,-60, Math.toRadians(180));

        Pose2d firstSpikeMark = new Pose2d(-41,-22.25, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder trajectoryOne = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(scoreRedBasketLeft, Math.toRadians(180))
                .waitSeconds(2)
                .splineTo(firstSpikeMark.position, Math.toRadians(180))
                .waitSeconds(2)
                .splineTo(scoreRedBasketLeft.position, Math.toRadians(180));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Action chosenTrajectory;
        chosenTrajectory = trajectoryOne.build();

        Actions.runBlocking(
                chosenTrajectory
        );
    }
}