package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Disabled
@Autonomous(name="Auton - Net Side - orig")
public class AutonNetSideOriginal extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-16,-62, Math.toRadians(90));
        Pose2d scoreChamber = new Pose2d(-8,-28, Math.toRadians(90));
        Pose2d scoreHighBasket = new Pose2d(-53, -49, Math.toRadians(225));
        Pose2d firstSpikeMark = new Pose2d(-33, -33, Math.toRadians(150));
        Pose2d secondSpikeMark = new Pose2d(-39, -30, Math.toRadians(150));
        Pose2d parkNearSubmersible = new Pose2d(-15, -6.5, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, drive.pose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoreChamber)
                .setReversed(true)
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(180));

        TrajectoryActionBuilder tab3  = drive.actionBuilder(firstSpikeMark)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(scoreHighBasket)
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(secondSpikeMark)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180));

        TrajectoryActionBuilder tab6 = drive.actionBuilder(scoreHighBasket)
                .setReversed(true)
                .splineToLinearHeading(parkNearSubmersible, Math.toRadians(0));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new SequentialAction(
                robot.setLights(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE),
                new ParallelAction(
                        robot.autonStart(),
                        tab1.build()
                ),
                robot.setLights(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE),
                robot.scoreSpecimen(),
                robot.setLights(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE),
                new ParallelAction(
                        new SequentialAction(
                                robot.goToReadyPose(),
                                robot.floorAcquire()
                        ),
                        tab2.build()
                ),
                robot.floorAcquireReach(),
                robot.setLights(RevBlinkinLedDriver.BlinkinPattern.VIOLET),
                new ParallelAction(
                        tab3.build(),
                        robot.prepareScoreHighBasket()
                ),
                robot.highBasketReach(),
                robot.setLights(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE),
                new ParallelAction(
                        new SleepAction(0.3),
                        robot.scoreHighBasket()
                ),
                robot.setLights(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE),
                new ParallelAction(
                        new SequentialAction(
                                robot.prepareScoreHighBasket(),
                                robot.goToReadyPose(),
                                robot.floorAcquire()
                        ),
                        new SequentialAction(
                                tab4.build(),
                                new SleepAction(0.5)
                        )
                ),
                robot.floorAcquireReach(),
                robot.setLights(RevBlinkinLedDriver.BlinkinPattern.VIOLET),
                new ParallelAction(
                        robot.prepareScoreHighBasket(),
                        tab5.build()
                ),
                robot.highBasketReach(),
                robot.setLights(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE),
                new ParallelAction(
                        new SleepAction(0.3),
                        robot.scoreHighBasket()
                ),
                robot.setLights(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE),
                new ParallelAction(
                        new SequentialAction(
                                robot.prepareScoreHighBasket(),
                                robot.goToReadyPose()
                        ),
                        tab6.build()
                ),
                robot.goToReadyPose(),
                robot.setLights(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE)
        ));
    }
}