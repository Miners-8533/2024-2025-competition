package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PoseConfig;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu;
import org.firstinspires.ftc.teamcode.subsystems.TrajectoryConfig;

@Autonomous(name="Auton Basic Trajectory", group="Testing")
public class AutonBasicTrajectory extends LinearOpMode {

    @Override
    public void runOpMode() {
        PoseConfig poseConfig = new PoseConfig();
        SelectionMenu.AllianceColor allianceColor = SelectionMenu.AllianceColor.RED;
        SelectionMenu.FieldStartPosition fieldStartPosition = SelectionMenu.FieldStartPosition.RIGHT;

        MecanumDrive drive = new MecanumDrive(hardwareMap, poseConfig.getInitialPose());
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(drive);

//        TrajectoryActionBuilder trajectoryOne = drive.actionBuilder(initialPose)
//                .splineToLinearHeading(scoreChamber, Math.toRadians(90))
//                .waitSeconds(2)
//                .setReversed(true)
//                .splineToLinearHeading(scoreRedBasketLeft, Math.toRadians(180))
//                .waitSeconds(2)
//                .splineTo(firstSpikeMark.position, Math.toRadians(180))
//                .waitSeconds(2)
//                .strafeTo(scoreRedBasketLeft.position)
//                .waitSeconds(2)
//                /*.splineToLinearHeading(initialPose, Math.toRadians(90))*/;

        // Wait for the game to start (driver presses START)
        waitForStart();

        Action initialTrajectory = trajectoryConfig.ScoreChamberTrajectory(allianceColor, fieldStartPosition);

        Actions.runBlocking(
                initialTrajectory
        );
    }
}