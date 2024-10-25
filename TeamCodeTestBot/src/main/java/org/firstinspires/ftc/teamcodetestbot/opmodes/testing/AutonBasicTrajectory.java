package org.firstinspires.ftc.teamcodetestbot.opmodes.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcodetestbot.roadrunner.MecanumDrive;

@Autonomous(name="Auton Basic Trajectory", group="Testing")
public class AutonBasicTrajectory extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-16,-63, Math.toRadians(90));

        Pose2d scoreChamber = new Pose2d(-8,-31, Math.toRadians(90));

        Pose2d scoreHighBasket = new Pose2d(-48, -48, Math.toRadians(225));

        Pose2d firstSpikeMark = new Pose2d(-35, -35, Math.toRadians(150));

        Pose2d secondSpikeMark = new Pose2d(-45, -35, Math.toRadians(150));

        Pose2d parkNearSubmersible = new Pose2d(-24, -6.5, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90))
                .waitSeconds(2)
                .setReversed(true)
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180))
                .waitSeconds(2)
                .splineToLinearHeading(parkNearSubmersible, Math.toRadians(0));
        // Wait for the game to start (driver presses START)
        waitForStart();

        Action chosenTrajectory;
        chosenTrajectory = tab1.build();

        Actions.runBlocking(
                chosenTrajectory
        );
    }
}
