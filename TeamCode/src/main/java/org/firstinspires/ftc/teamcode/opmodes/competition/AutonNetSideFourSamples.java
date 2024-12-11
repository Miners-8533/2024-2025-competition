package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="Auton - Net Side - 4 Samples", group="Competition")
public class AutonNetSideFourSamples extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(-16,-62, Math.toRadians(90));
        Pose2d scoreHighBasket = new Pose2d(-55, -47, Math.toRadians(225));
        Pose2d firstSpikeMark = new Pose2d(-34.5, -21.5, Math.toRadians(180));
        Pose2d secondSpikeMark = new Pose2d(-44.5, -20.5, Math.toRadians(180));
        Pose2d thirdSpikeMark = new Pose2d(-52.5, -20, Math.toRadians(175));
        Pose2d parkNearSubmersible = new Pose2d(-15, -6.5, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, initialPose);

        TrajectoryActionBuilder scoreFirstSampleTab = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder firstSpikeMarkTab = drive.actionBuilder(scoreHighBasket)
                .setReversed(true)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(90),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder scoreSecondSampleTab  = drive.actionBuilder(firstSpikeMark)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180));

        TrajectoryActionBuilder secondSpikeMarkTab = drive.actionBuilder(scoreHighBasket)
                .setTangent(0)
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180),null, new ProfileAccelConstraint(-15,50));

        TrajectoryActionBuilder scoreThirdSampleTab = drive.actionBuilder(secondSpikeMark)
                .setReversed(true)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180),null, new ProfileAccelConstraint(-40,40));

        TrajectoryActionBuilder parkNearSubmersibleTab = drive.actionBuilder(scoreHighBasket)
                .setReversed(true)
                .splineToLinearHeading(parkNearSubmersible, Math.toRadians(0),null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder thirdSpikeMarkTab = drive.actionBuilder(scoreHighBasket)
                .setTangent(0)
                .splineToLinearHeading(thirdSpikeMark,Math.toRadians(180),null, new ProfileAccelConstraint(-15,50));

        TrajectoryActionBuilder scoreFourthSampleTab = drive.actionBuilder(thirdSpikeMark)
                .setReversed(true)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180),null, new ProfileAccelConstraint(-40,40));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
                robot.startGrip(),
                new SleepAction(0.5),
                new ParallelAction(
                        scoreFirstSampleTab.build(),
                        robot.prepareScoreHighBasket()
                ),
                robot.highBasketReach(),
                new ParallelAction(
                        new SleepAction(0.5),
                        robot.scoreHighBasket()
                ),
                new ParallelAction(
                        new SequentialAction(
                                robot.prepareScoreHighBasket(),
                                robot.goToReadyPose(),
                                new SleepAction(0.5),
                                robot.floorHover()
                        ),
                        firstSpikeMarkTab.build()
                ),
                new ParallelAction(
                        new SleepAction(0.1),
                        robot.floorAcquire()
                ),
                robot.floorAcquireReach(),
                new ParallelAction(
                        scoreSecondSampleTab.build(),
                        robot.prepareScoreHighBasket()
                ),
                robot.highBasketReach(),
                new ParallelAction(
                        new SleepAction(0.5),
                        robot.scoreHighBasket()
                ),
                robot.prepareScoreHighBasket(),
                new ParallelAction(
                        new SequentialAction(
                                robot.goToReadyPose(),
                                new SleepAction(0.5),
                                robot.floorHover()
                        ),
                        secondSpikeMarkTab.build()
                ),
                new ParallelAction(
                        new SleepAction(0.1),
                        robot.floorAcquire()
                ),
                robot.floorAcquireReach(),
                new ParallelAction(
                        robot.prepareScoreHighBasket(),
                        scoreThirdSampleTab.build()
                ),
                robot.highBasketReach(),
                new ParallelAction(
                        new SleepAction(0.5),
                        robot.scoreHighBasket()
                ),
                robot.prepareScoreHighBasket(),
                new ParallelAction(
                        new SequentialAction(
                                robot.goToReadyPose(),
                                new SleepAction(0.5),
                                robot.floorHover()
                        ),
                        thirdSpikeMarkTab.build()
                ),
                new ParallelAction(
                        new SleepAction(0.1),
                        robot.floorAcquire()
                ),
                robot.floorAcquireReach(),
                new SleepAction(0.2),
                new ParallelAction(
                        robot.prepareScoreHighBasket(),
                        scoreFourthSampleTab.build()
                ),
                robot.highBasketReach(),
                new ParallelAction(
                        new SleepAction(0.5),
                        robot.scoreHighBasket()
                ),
                robot.prepareScoreHighBasket(),
                new ParallelAction(
                        robot.goToReadyPose(),
                        parkNearSubmersibleTab.build()
                ),
                robot.goToReadyPose()
        )));
    }
}
