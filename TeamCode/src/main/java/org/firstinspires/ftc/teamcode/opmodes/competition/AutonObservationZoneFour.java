package org.firstinspires.ftc.teamcode.opmodes.competition;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.onbotjava.handlers.file.NewFile;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SubSystemConfigs;

@Autonomous(name="Observation - 4", group="Competition")
public class AutonObservationZoneFour extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(16,-62, Math.toRadians(90));
        Pose2d scoreChamber = new Pose2d(8,-27, Math.toRadians(90));
        Pose2d intermediatePose = new Pose2d(30, -36,Math.toRadians(30));
        Pose2d firstSpikeMark = new Pose2d(38, -30, Math.toRadians(300));
        Pose2d secondSpikeMark = new Pose2d(45, -23, Math.toRadians(270));
        Pose2d observationZonePose = new Pose2d(42, -58, Math.toRadians(270));
        Pose2d secondObservationZonePose = new Pose2d(52, -65, Math.toRadians(270));
        Pose2d acquireSpecimenPose = new Pose2d(41, -65, Math.toRadians(270));
        Pose2d chamberTwoPose = new Pose2d(7,-22, Math.toRadians(90));
        Pose2d chamberThreePose = new Pose2d(5.5,-22, Math.toRadians(90));
        Pose2d chamberFourPose = new Pose2d(3.5, -22, Math.toRadians(90));
        Pose2d parkChamberPose = new Pose2d(-5, -21,Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, drive.pose);

        TrajectoryActionBuilder scoreChamberTab = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder firstSpikeMarkTab = drive.actionBuilder(scoreChamber)
                .setReversed(true)
                .splineToLinearHeading(intermediatePose, Math.toRadians(0))
                .splineToSplineHeading(firstSpikeMark, Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(observationZonePose,Math.toRadians(270))
                .setReversed(true)
                .splineToConstantHeading(secondSpikeMark.position, Math.toRadians(0))
                .splineToSplineHeading(secondObservationZonePose, Math.toRadians(270),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder secondScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToSplineHeading(chamberTwoPose, Math.toRadians(90), null, new ProfileAccelConstraint(-40,40));

        TrajectoryActionBuilder acquireThirdSpecimen = drive.actionBuilder(chamberTwoPose)
                .setReversed(true)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270),null, new ProfileAccelConstraint(-35, 50));

        TrajectoryActionBuilder acquireFourthSpecimen = drive.actionBuilder(chamberThreePose)
                .setReversed(true)
                .splineToLinearHeading(acquireSpecimenPose, Math.toRadians(270),null, new ProfileAccelConstraint(-35, 50));

        TrajectoryActionBuilder thirdScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToSplineHeading(chamberThreePose, Math.toRadians(90), null, new ProfileAccelConstraint(-35,40));

        TrajectoryActionBuilder fourthScoreChamberTab = drive.actionBuilder(acquireSpecimenPose)
                .setReversed(true)
                .splineToSplineHeading(chamberFourPose, Math.toRadians(90),null, new ProfileAccelConstraint(-25,40));

        TrajectoryActionBuilder parkChamberTab = drive.actionBuilder(chamberFourPose)
                .strafeTo(parkChamberPose.position);

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
                        new SleepAction(0.4)
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
                        new SleepAction(0.4)
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
                        new SleepAction(0.4)
                ),
                new ParallelAction(
                        robot.autonStart(),
                        fourthScoreChamberTab.build()
                ),
                new ParallelAction(
                        parkChamberTab.build(),
                        new SequentialAction(
                                robot.scoreSpecimen(),
                                robot.goToReadyPose()
                        )
                )
        )));
    }
}
