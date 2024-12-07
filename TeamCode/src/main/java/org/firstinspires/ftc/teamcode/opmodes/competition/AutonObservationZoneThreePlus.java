package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SubSystemConfigs;

@Autonomous(name="Auton - Observation Side - 3 Specimen plus", group="Competition")
public class AutonObservationZoneThreePlus extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(16,-62, Math.toRadians(90));
        Pose2d scoreChamber = new Pose2d(8,-27, Math.toRadians(90));
        Pose2d intermediatePose = new Pose2d(30, -36,Math.toRadians(30));
        Pose2d firstSpikeMark = new Pose2d(38, -30, Math.toRadians(300));
        Pose2d secondSpikeMark = new Pose2d(48, -30, Math.toRadians(300));
        Pose2d thirdSpikeMark = new Pose2d(58, -30, Math.toRadians(300));
        Pose2d observationZonePose = new Pose2d(47, -55, Math.toRadians(270));
        Pose2d acquireSpecimenPose = new Pose2d(47, initialPose.position.y, Math.toRadians(270));
        Pose2d chamberTwoPose = new Pose2d(5,-27, Math.toRadians(90));
        Pose2d chamberThreePose = new Pose2d(2,-27, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, drive.pose);

        TrajectoryActionBuilder scoreChamberTab = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90));

        TrajectoryActionBuilder firstSpikeMarkTab = drive.actionBuilder(scoreChamber)
                .setReversed(true)
                .splineToLinearHeading(intermediatePose, Math.toRadians(0))
                .splineToSplineHeading(firstSpikeMark, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(observationZonePose,Math.toRadians(270))
                .setReversed(true)
                .splineToLinearHeading(secondSpikeMark, Math.toRadians(270))
                .splineToSplineHeading(observationZonePose, Math.toRadians(270))
                .splineToSplineHeading(thirdSpikeMark, Math.toRadians(270))
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270));

        TrajectoryActionBuilder secondScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToSplineHeading(chamberTwoPose, Math.toRadians(90),null, new ProfileAccelConstraint(-40,40));

        TrajectoryActionBuilder acquireThirdSpecimen = drive.actionBuilder(chamberTwoPose)
                .setReversed(true)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270));

        TrajectoryActionBuilder thirdScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToLinearHeading(chamberThreePose, Math.toRadians(90),null, new ProfileAccelConstraint(-40,40));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
                new ParallelAction(
                        robot.autonStart(),
                        scoreChamberTab.build()
                ),
                robot.scoreSpecimen(),
                robot.goToReadyPose(),
                new ParallelAction(
                        firstSpikeMarkTab.build(),
                        new SequentialAction(
                                new SleepAction(1.0),
                                robot.setWing(SubSystemConfigs.WING_DOWN)
                        )
                ),
                robot.setWing(SubSystemConfigs.WING_UP),
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
