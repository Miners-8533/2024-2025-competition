package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public boolean lastAcquire;
    public boolean isOutakeSample;
    public boolean isReady;
    public boolean isScoreSpecimen;
    public boolean isScoreBasket;
    public double reachScrub;
    public double liftScrub;
    public boolean isBumperDown;
    public boolean isGyroReset;
    public boolean isTargetSample;
    public boolean isWingDown;

    public ElapsedTime acquireTimer;
    public DriveStation(Gamepad driverController, Gamepad operatorController) {
        driver = driverController;
        operator = operatorController;
        acquireTimer = new ElapsedTime();
        lastAcquire = false;
    }

    public void update() {
        //driver move
        forward = -driver.left_stick_y;
        strafe = -driver.left_stick_x;
        rotation = -driver.right_stick_x / 2.0;
        isGyroReset = driver.back;

        //driver score
        isScoreSpecimen = driver.a;
        isOutakeSample = driver.a;

        //driver reset
        isReady = driver.y;

        //operator climb
        isClimb = operator.dpad_up;
        isClimbPrep = operator.dpad_right || operator.dpad_left;
        isClimbReset = operator.dpad_down;

        //operator acquire
        isAquireSpecimen = operator.right_bumper;
        lastAcquire = isAquireSample;
        isAquireSample = operator.a;
        isTargetSample = operator.right_trigger > 0.5;
        if(isAquireSample != lastAcquire) {
            acquireTimer.reset();
        }

        //operator bumper override
        isBumperDown = operator.left_trigger > 0.5;

        isWingDown = operator.left_bumper;

        //operator prepare for high basket
        isScoreBasket = operator.y;

        //operator scrubs
        reachScrub = operator.left_stick_y;
        liftScrub = operator.right_stick_y;
    }
}
