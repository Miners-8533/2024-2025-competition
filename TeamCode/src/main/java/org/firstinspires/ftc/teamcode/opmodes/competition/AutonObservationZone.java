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

@Autonomous(name="Auton - Observation Side", group="Competition")
public class AutonObservationZone extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(16,-62, Math.toRadians(90));
        Pose2d scoreChamber = new Pose2d(8,-27, Math.toRadians(90));
        Pose2d intermediatePose = new Pose2d(32, -40,Math.toRadians(90));
        Pose2d firstSpikeMark = new Pose2d(34.5, -27.5, Math.toRadians(0));
        Pose2d secondSpikeMark = new Pose2d(44.5, -26, Math.toRadians(0));
        Pose2d thirdSpikeMark = new Pose2d(52.5, -25.5, Math.toRadians(0));
        Pose2d observationZonePose = new Pose2d(47, -48, Math.toRadians(315));
        Pose2d secondObservationZonePose = new Pose2d(57, -48, Math.toRadians(315));
        Pose2d thirdObservationZonePose = new Pose2d(67, -48, Math.toRadians(315));
        Pose2d preAcquireSpecimen = new Pose2d(47, -55, Math.toRadians(270));
        Pose2d acquireSpecimenPose = new Pose2d(47, -65, Math.toRadians(270));
        Pose2d chamberTwoPose = new Pose2d(5,-23, Math.toRadians(90));
        Pose2d chamberThreePose = new Pose2d(2,-23, Math.toRadians(90));
        Pose2d chamberFourPose = new Pose2d(-1, -23, Math.toRadians(90));
        Pose2d parkOnWall = new Pose2d(47, -65, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, drive.pose);

        TrajectoryActionBuilder scoreChamberTab = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder driveToIntermediateTab = drive.actionBuilder(scoreChamber)
                .setReversed(true)
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(intermediatePose,Math.toRadians(90));

        TrajectoryActionBuilder firstSpikeMarkTab = drive.actionBuilder(scoreChamber)
                .setReversed(true)
                .setTangent(Math.toRadians(270.0))
//                .splineToLinearHeading(intermediatePose, Math.toRadians(180))
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(90),null, new ProfileAccelConstraint(-30,50));

        TrajectoryActionBuilder firstObservationTab = drive.actionBuilder(firstSpikeMark)
                .splineToLinearHeading(observationZonePose, Math.toRadians(90));

        TrajectoryActionBuilder secondSpikeMarkTab = drive.actionBuilder(observationZonePose)
                .setTangent(180)
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180),null, new ProfileAccelConstraint(-20,50));

        TrajectoryActionBuilder thirdSpikeMarkTab = drive.actionBuilder(secondObservationZonePose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(thirdSpikeMark, Math.toRadians(0));

        TrajectoryActionBuilder secondObservationZoneTab = drive.actionBuilder(secondSpikeMark)
                .splineToLinearHeading(secondObservationZonePose, Math.toRadians(90));

//        TrajectoryActionBuilder thirdObservationZoneTab = drive.actionBuilder(thirdSpikeMark)
//                .lineToY(thirdObservationZonePose.position.y);

        TrajectoryActionBuilder acquireSecondSpecimenTab = drive.actionBuilder(secondObservationZonePose)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(0), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder secondScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToLinearHeading(chamberTwoPose, Math.toRadians(90), null, new ProfileAccelConstraint(-40,40));

        TrajectoryActionBuilder acquireThirdSpecimen = drive.actionBuilder(chamberTwoPose)
                .setReversed(true)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270));

        TrajectoryActionBuilder acquireFourthSpecimen = drive.actionBuilder(chamberThreePose)
                .setReversed(true)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270));

        TrajectoryActionBuilder thirdScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToLinearHeading(chamberThreePose, Math.toRadians(90), null, new ProfileAccelConstraint(-40,40));
        TrajectoryActionBuilder fouthScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToLinearHeading(chamberFourPose, Math.toRadians(90), null, new ProfileAccelConstraint(-40,40));

        TrajectoryActionBuilder parkInObservationZone = drive.actionBuilder(chamberFourPose)
                .setReversed(true)
                .splineToLinearHeading(parkOnWall, Math.toRadians(270), null, new ProfileAccelConstraint(-100,100));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
                new ParallelAction(
                        robot.autonStart(),
                        scoreChamberTab.build()
                ),
                robot.scoreSpecimen(),
                robot.goToReadyPose(),
                firstSpikeMarkTab.build(),
                new ParallelAction(
                        new SleepAction(1.0),
                        robot.floorAcquire()
                ),
                robot.floorAcquireReach(),
                robot.goToReadyPose(),
                firstObservationTab.build(),
                new SequentialAction(
                        robot.outakeSampleGround(),
                        new SleepAction(0.3)
                ),
                robot.goToReadyPose(),
                secondSpikeMarkTab.build(),
                new ParallelAction(
                        new SleepAction(1.0),
                        robot.floorAcquire()
                ),
                robot.floorAcquireReach(),
                robot.goToReadyPose(),
                secondObservationZoneTab.build(),
                new SequentialAction(
                        robot.outakeSampleGround(),
                        new SleepAction(0.3)
                ),
                robot.goToReadyPose(),
                acquireSecondSpecimenTab.build(),
                new ParallelAction(
                        robot.acquireSpecimen(),
                        new SleepAction(0.3)
                ),
                new ParallelAction(
                        robot.autonStart(),
                        secondScoreChamberTab.build()
                ),
                robot.scoreSpecimen(),
                new ParallelAction(
                        acquireThirdSpecimen.build(),
                        robot.goToReadyPose()
                ),
                new ParallelAction(
                        robot.acquireSpecimen(),
                        new SleepAction(0.3)
                ),
                new ParallelAction(
                        robot.autonStart(),
                        thirdScoreChamberTab.build()
                ),
                robot.scoreSpecimen(),
                robot.goToReadyPose()
        )));
    }
}
