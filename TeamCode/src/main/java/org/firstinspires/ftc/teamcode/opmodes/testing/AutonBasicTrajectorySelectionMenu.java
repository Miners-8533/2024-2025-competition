package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.MenuState;
import org.firstinspires.ftc.teamcode.subsystems.SelectionMenu.FieldStartPosition;

@Autonomous(name="Auton Basic Trajectory", group="Testing")
public class AutonBasicTrajectorySelectionMenu extends LinearOpMode {

    SelectionMenu selectionMenu = new SelectionMenu(this,telemetry);

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
        double startDelay = selectionMenu.getStartDelay();
        Pose2d initialPose;



        switch(allianceColor){
            case RED:
                if(fieldStartPosition == FieldStartPosition.RIGHT) {
                    initialPose = new Pose2d(24, -24, 90);
                } else {
                    initialPose = new Pose2d(-24, -24, 90);
                }
                break;
            case BLUE:
            default:
                if(fieldStartPosition == FieldStartPosition.RIGHT) {
                    initialPose = new Pose2d(-24, 24, 270);
                } else {
                    initialPose = new Pose2d(24, 24, 270);
                }
                break;
        }

//        Pose2d startPose = trajectoryConfig.getStartPose(allianceColor, stagePosition);

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(24)
                .lineToX(-24);

        // Wait for the game to start (driver presses START)
        waitForStart();

        Action chosenTrajectory;
        chosenTrajectory = tab1.build();

        Actions.runBlocking(
                chosenTrajectory
        );
    }
}