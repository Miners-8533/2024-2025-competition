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
    public DriveStation(Gamepad driverController, Gamepad operatorController) {
        driver = driverController;
        operator = operatorController;
    }

    public void update() {
        forward = -driver.left_stick_y;
        strafe = -driver.left_stick_x;
        rotation = -driver.right_stick_x;

        isClimb = driver.dpad_up;
        isClimbPrep = driver.dpad_right;
        isClimbReset = driver.dpad_down;

        isAquireSpecimen = driver.right_bumper;
        isScoreSpecimen = driver.left_bumper;

        isAquireSample = driver.a;
        isOutakeSample = driver.b;
        isScoreBasket = driver.x;

        isReady = driver.y;

        reachScrub = driver.right_trigger;
    }
}
