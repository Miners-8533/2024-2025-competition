package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveStation {
    private Gamepad driver;
    private Gamepad operator;

    public double forward;
    public double strafe;
    public double rotation;
    public boolean isClimbPrep;
    public boolean isClimb;
    public boolean isClimbReset;
    public boolean isAquireSpecimen;
    public boolean isAquireSample;
    public boolean isOutakeSample;
    public boolean isReady;
    public boolean isScoreSpecimen;
    public boolean isScoreBasket;
    public double reachScrub;
    public double liftScrub;
    public boolean isBumperDown;
    public boolean isGyroReset;
    public DriveStation(Gamepad driverController, Gamepad operatorController) {
        driver = driverController;
        operator = operatorController;
    }

    public void update() {
        //driver move
        forward = -driver.left_stick_y;
        strafe = -driver.left_stick_x;
        rotation = -driver.right_stick_x / 2.0;
        isGyroReset = driver.start;

        //driver score
        isScoreSpecimen = driver.a;
        isOutakeSample = driver.a;

        //driver reset
        isReady = driver.y;

        //operator climb
        isClimb = operator.dpad_up;
        isClimbPrep = operator.dpad_right;
        isClimbReset = operator.dpad_down;

        //operator acquire
        isAquireSpecimen = operator.right_bumper;
        isAquireSample = operator.a;

        //operator bumper overide
        isBumperDown = operator.start;

        //operator prepare for high basket
        isScoreBasket = operator.y;

        //operator scrubs
        reachScrub = operator.left_stick_y;
        liftScrub = operator.right_stick_y;
    }
}
