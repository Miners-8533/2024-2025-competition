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
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="Auton - Observation Side - four specimens", group="Testing")
public class AutonObservationFourSpecimens extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(16,-62, Math.toRadians(90));
        Pose2d scoreChamber = new Pose2d(8,-27, Math.toRadians(90));
        Pose2d intermediatePose = new Pose2d(35, -28,Math.toRadians(90));
        Pose2d firstSpikeMark = new Pose2d(45, -12, Math.toRadians(90));
        Pose2d secondSpikeMark = new Pose2d(55, -12, Math.toRadians(90));
        Pose2d thirdSpikeMark = new Pose2d(68, -12, Math.toRadians(90));
        Pose2d observationZonePose = new Pose2d(47, -55, Math.toRadians(90));
        Pose2d secondObservationZonePose = new Pose2d(57, -55, Math.toRadians(90));
        Pose2d thirdObservationZonePose = new Pose2d(67, -55, Math.toRadians(90));
        Pose2d preAcquireSpecimen = new Pose2d(47, -55, Math.toRadians(270));
        Pose2d acquireSpecimenPose = new Pose2d(47, initialPose.position.y, Math.toRadians(270));
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

        TrajectoryActionBuilder firstSpikeMarkTab = drive.actionBuilder(intermediatePose)
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(0));

        TrajectoryActionBuilder firstObservationTab = drive.actionBuilder(firstSpikeMark)
                .lineToY(observationZonePose.position.y);

        TrajectoryActionBuilder secondSpikeMarkTab = drive.actionBuilder(scoreChamber)
                .setTangent(Math.toRadians(300))
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(0));

        TrajectoryActionBuilder thirdSpikeMarkTab = drive.actionBuilder(secondObservationZonePose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(thirdSpikeMark, Math.toRadians(0));

        TrajectoryActionBuilder secondObservationZoneTab = drive.actionBuilder(secondSpikeMark)
                .lineToY(secondObservationZonePose.position.y);

        TrajectoryActionBuilder thirdObservationZoneTab = drive.actionBuilder(thirdSpikeMark)
                .lineToY(thirdObservationZonePose.position.y);

        TrajectoryActionBuilder acquireSecondSpecimenTab = drive.actionBuilder(thirdObservationZonePose)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270));

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
                .splineToLinearHeading(parkOnWall, Math.toRadians(270));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
                new ParallelAction(
                        robot.autonStart(),
                        scoreChamberTab.build()
                ),
                robot.scoreSpecimen(),
                new ParallelAction(
                        robot.goToReadyPose(),
                        secondSpikeMarkTab.build()
                ),
                secondObservationZoneTab.build(),
                thirdSpikeMarkTab.build(),
                thirdObservationZoneTab.build(),
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
                new ParallelAction(
                        acquireFourthSpecimen.build(),
                        robot.goToReadyPose()
                ),
                new ParallelAction(
                        robot.acquireSpecimen(),
                        new SleepAction(0.3)
                ),
                new ParallelAction(
                        robot.autonStart(),
                        fouthScoreChamberTab.build()
                ),
                robot.scoreSpecimen(),
                parkInObservationZone.build(),
                robot.setLastPose(drive)
        )));
    }
}
