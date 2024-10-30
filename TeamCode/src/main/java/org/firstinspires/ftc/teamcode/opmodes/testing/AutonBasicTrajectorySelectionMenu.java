package org.firstinspires.ftc.teamcode.opmodes.testing;

import static org.firstinspires.ftc.teamcode.subsystems.TrajectoryConfig.getTrajectory;
import static org.firstinspires.ftc.teamcode.subsystems.TrajectoryConfig.mirrorPose;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.MenuState;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.FieldStartPosition;
import org.firstinspires.ftc.teamcode.subsystems.PoseConfig;
import org.firstinspires.ftc.teamcode.subsystems.TrajectoryConfig;

@Autonomous(name="Auton Basic Trajectory w/Menu", group="Testing")
public class AutonBasicTrajectorySelectionMenu extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    SelectionMenu selectionMenu = new SelectionMenu(this,telemetry);
    PoseConfig poseConfig = new PoseConfig();

    @Override
    public void runOpMode() {
        telemetry.setAutoClear(false);
        while(!isStarted()) {
            selectionMenu.displayMenu();

            // Check for user input
            if (gamepad1.dpad_up) {
                selectionMenu.navigateUp();
            } else if (gamepad1.dpad_down) {
                selectionMenu.navigateDown();
            } else if (gamepad1.a) {
                selectionMenu.selectOption();
            } else if (gamepad1.b) {
                selectionMenu.navigateBack();
            }

            idle();
        }
        selectionMenu.setMenuState(MenuState.READY);
        selectionMenu.displayMenu();

        AllianceColor allianceColor = selectionMenu.getAllianceColor();
        FieldStartPosition fieldStartPosition = selectionMenu.getFieldStartPosition();
        String targetQuadrant = selectionMenu.getTargetQuadrant();
        double startDelay = selectionMenu.getStartDelay();
        Pose2d correctedInitialPose = mirrorPose(poseConfig.getInitialPose().position.x, poseConfig.getInitialPose().position.y, poseConfig.getInitialPose().heading.toDouble(), targetQuadrant);

        MecanumDrive drive = new MecanumDrive(hardwareMap,correctedInitialPose);

        // Wait for the game to start (driver presses START)
        waitForStart();
        if (opModeIsActive()) {

            runtime.reset();

            while (opModeIsActive() && runtime.seconds() < startDelay) {
                telemetry.clear();
                telemetry.addData("Status", "Delaying for " + startDelay + " seconds...");
                telemetry.addData("Run time", "Seconds - " + runtime.seconds());
                telemetry.update();
            }

            if (fieldStartPosition == FieldStartPosition.LEFT) {
                Actions.runBlocking(new SequentialAction(
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getInitialPose(), poseConfig.getChamberPose(), 90, false, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getChamberPose(), poseConfig.getFirstSpikeMarkPose(), 180, true, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getFirstSpikeMarkPose(), poseConfig.getBasketPose(), 180, false, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getBasketPose(), poseConfig.getSecondSpikeMarkPose(), 180, false, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getSecondSpikeMarkPose(), poseConfig.getBasketPose(), 180, false, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getBasketPose(), poseConfig.getParkSubmersiblePose(), 0, true, "SPLINE")
                ));
            } else {
                Actions.runBlocking(new SequentialAction(
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getInitialPose(), poseConfig.getChamberPose(), 90, false, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getChamberPose(), poseConfig.getFirstSpikeMarkPose(), 90, true, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getFirstSpikeMarkPose(), poseConfig.getObservationZonePose(), 270, false, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getObservationZonePose(), poseConfig.getSecondSpikeMarkPose(), 180, true, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getSecondSpikeMarkPose(), poseConfig.getObservationZonePose(), 180, false, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getObservationZonePose(), new Pose2d(poseConfig.getObservationZonePose().position.x, poseConfig.getObservationZonePose().position.y, Math.toRadians(270)), 180, false, "TURN"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, new Pose2d(poseConfig.getObservationZonePose().position.x, poseConfig.getObservationZonePose().position.y, Math.toRadians(270)), new Pose2d(poseConfig.getObservationZonePose().position.x, poseConfig.getInitialPose().position.y, Math.toRadians(270)), 180, false, "LINETOY"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, new Pose2d(poseConfig.getObservationZonePose().position.x, poseConfig.getInitialPose().position.y, Math.toRadians(270)), poseConfig.getChamberTwoPose(), 90, true, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getChamberTwoPose(), poseConfig.getObservationZonePose(), 270, true, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getObservationZonePose(), new Pose2d(poseConfig.getObservationZonePose().position.x, poseConfig.getObservationZonePose().position.y, Math.toRadians(270)), 180, false, "TURN"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, new Pose2d(poseConfig.getObservationZonePose().position.x, poseConfig.getObservationZonePose().position.y, Math.toRadians(270)), new Pose2d(poseConfig.getObservationZonePose().position.x, poseConfig.getInitialPose().position.y, Math.toRadians(270)), 180, false, "LINETOY"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, new Pose2d(poseConfig.getObservationZonePose().position.x, poseConfig.getInitialPose().position.y, Math.toRadians(270)), poseConfig.getChamberThreePose(), 90, true, "SPLINE"),
                        getTrajectory(drive, allianceColor, fieldStartPosition, poseConfig.getChamberThreePose(), poseConfig.getObservationZonePose(), 270, true, "SPLINE")
                ));
            }
        }
    }
}