package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SubSystemConfigs {

    public static final int LIFT_HOME_POS = -50;
    public static final int LIFT_HIGH_BASKET_POS = -5850;
    public static final int LIFT_LOW_BASKET_POS = -3000;
    public static final int LIFT_HIGH_CHAMBER_POS = -2750;
    public static final int LIFT_HIGH_CHAMBER_AUTON_INIT_POS = -2765;
    public static final int LIFT_HIGH_CHAMBER_AUTON_POS = -3000;
    public static final int LIFT_HIGH_CHAMBER_SCORE_POS = -1850;
    public static final int LIFT_HIGH_CHAMBER_SCORE_AUTO_POS = -1800;
    public static final int REACH_HOME_POS = 0;
    public static final int REACH_FULL_EXTEND_POS = -1500;
    public static final int REACH_FLOOR_EXTEND_POS = -1000; //-1200
    public static final int REACH_HIGH_BASKET_EXTEND_POS = -600;
    public static final double ELBOW_UP_POS = 0.18;
    public static final double ELBOW_READY_POS = 0.20; // 0.21
    public static final double ELBOW_SCORE_BASKET_POS = 0.38;
    public static final double ELBOW_ACQUIRE_POS = 0.64;//.64
    public static final double WHEEL_STOP_SPD = 0.53;
    public static final double WHEEL_ACQUIRE_SPD = 1.0;
    public static final double WHEEL_SCORE_SPD = 0.0;
    public static final double WHEEL_HOLD_SPD = 0.53;
    public static final double GRIPPER_OPEN_POS = 0.0;
    public static final double GRIPPER_CLOSED_POS = 0.9;
    public static final double GRIPPER_HOLD_POS = 0.9;
    public static final int CLIMB_HOME = -50;
    public static final int CLIMB_PRE_CLIMB = -5500;
    public static final int CLIMB_ASCENT_1 = -10950;
    public static final double BUMPER_UP = 0.0;
    public static final double BUMPER_DOWN = 1.0;
    public static final int REACH_SCRUB_SPD = 50;
    public static final int REACH_SCRUB_SPD_AUTON = 15;
    public static final int LIFT_SCRUB_SPD = 50;
    public static final int LIFT_FFC_TOLERANCE = 50;
    public static final int FFC_TOLERANCE = 30;


    public static final MotorWithEncoderAndController.Config liftConfig =
            new MotorWithEncoderAndController.Config(
                "lift",
                new PIDFCoefficients(0.02,0.0,0.0,0.25),
                0.04,
                DcMotorSimple.Direction.FORWARD,
                LIFT_FFC_TOLERANCE
            );
    public static final MotorWithEncoderAndController.Config reachConfig =
            new MotorWithEncoderAndController.Config(
                "reach",
                new PIDFCoefficients(0.006,0.0,0.0,0.0),
                0.08,
                DcMotorSimple.Direction.FORWARD,
                FFC_TOLERANCE
            );
    public static final MotorWithEncoderAndController.Config climbConfig =
            new MotorWithEncoderAndController.Config(
                "climber",
                new PIDFCoefficients(0.002,0.0,0.0,0.0),
                0.08,
                DcMotorSimple.Direction.FORWARD,
                FFC_TOLERANCE
            );
}
