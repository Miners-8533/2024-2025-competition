package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Disabled
@Autonomous(name="Auton - Action FF - TEST", group="Test")
public class AutonVerifyFFAction extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(16,-62, Math.toRadians(90));
        Pose2d scoreChamber = new Pose2d(8,-27, Math.toRadians(90));
        Pose2d firstSpikeMark = new Pose2d(33, -35, Math.toRadians(27.5));
        Pose2d secondSpikeMark = new Pose2d(39, -35, Math.toRadians(27.5));
        Pose2d observationZonePose = new Pose2d(47, -58, Math.toRadians(315));
        Pose2d chamberTwoPose = new Pose2d(5,-27, Math.toRadians(90));
        Pose2d chamberThreePose = new Pose2d(2,-27, Math.toRadians(90));
        Pose2d parkOnWall = new Pose2d(47, -65, Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, initialPose);

        TrajectoryActionBuilder scoreChamberTab = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90));

        TrajectoryActionBuilder firstSpikeMarkTab = drive.actionBuilder(scoreChamber)
                .setReversed(true)
                .splineToLinearHeading(firstSpikeMark, Math.toRadians(90));

        TrajectoryActionBuilder firstObservationTab = drive.actionBuilder(firstSpikeMark)
                .splineToLinearHeading(observationZonePose, Math.toRadians(270));

        TrajectoryActionBuilder secondSpikeMarkTab = drive.actionBuilder(observationZonePose)
                .setReversed(true)
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180));

        TrajectoryActionBuilder secondObservationZoneTab = drive.actionBuilder(secondSpikeMark)
                .splineToLinearHeading(observationZonePose, Math.toRadians(180));

        TrajectoryActionBuilder turnToSpecimenTab = drive.actionBuilder(observationZonePose)
                .turnTo(Math.toRadians(270))
                .lineToY(parkOnWall.position.y);

        TrajectoryActionBuilder secondScoreChamberTab = drive.actionBuilder(new Pose2d(observationZonePose.position.x, initialPose.position.y, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(chamberTwoPose, Math.toRadians(90));

        TrajectoryActionBuilder thirdObservationZoneTab = drive.actionBuilder(chamberTwoPose)
                .setReversed(true)
                .splineToLinearHeading(observationZonePose, Math.toRadians(270));

        TrajectoryActionBuilder thirdScoreChamberTab = drive.actionBuilder(new Pose2d(observationZonePose.position.x, initialPose.position.y, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(chamberThreePose, Math.toRadians(90));

        TrajectoryActionBuilder parkInObservationZone = drive.actionBuilder(chamberThreePose)
                .setReversed(true)
                .splineToLinearHeading(parkOnWall, Math.toRadians(270));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new SequentialAction(

        ));
    }
}
