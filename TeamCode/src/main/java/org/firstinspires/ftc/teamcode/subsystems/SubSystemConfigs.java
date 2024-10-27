package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class SubSystemConfigs {

    public static final int LIFT_HOME_POS = 0;
    public static final int LIFT_HIGH_BASKET_POS = 0;
    public static final int LIFT_HIGH_CHAMBER_POS = 0;
    public static final int REACH_HOME_POS = 0;
    public static final int REACH_FULL_EXTEND_POS = 0;
    public static final double ELBOW_UP_POS = 0.0;
    public static final double ELBOW_READY_POS = 0.0;
    public static final double ELBOW_SCORE_BASKET_POS = 0.0;
    public static final double ELBOW_AQUIRE_POS = 0.0;
    public static final double WHEEL_STOP_SPD = 0.0;
    public static final double WHEEL_AQUIRE_SPD = 0.0;
    public static final double WHEEL_SCORE_SPD = 0.0;
    public static final double WHEEL_HOLD_SPD = 0.0;
    public static final double GRIPPER_OPEN_POS = 0.0;
    public static final double GRIPPER_CLOSED_POS = 0.0;
    public static final double GRIPPER_HOLD_POS = 0.0;

    public static final MotorWithEncoderAndController.Config liftConfig =
            new MotorWithEncoderAndController.Config(
                "lift",
                new PIDFCoefficients(0.0,0.0,0.0,0.0),
                0.0,
                DcMotorSimple.Direction.FORWARD
            );
    public static final MotorWithEncoderAndController.Config reachConfig =
            new MotorWithEncoderAndController.Config(
                "reach",
                new PIDFCoefficients(0.0,0.0,0.0,0.0),
                0.0,
                DcMotorSimple.Direction.FORWARD
            );
    public static final MotorWithEncoderAndController.Config climbConfig =
            new MotorWithEncoderAndController.Config(
                "climb",
                new PIDFCoefficients(0.0,0.0,0.0,0.0),
                0.0,
                DcMotorSimple.Direction.FORWARD
            );
    public enum GantryStates {
        START(LIFT_HOME_POS,REACH_HOME_POS,ELBOW_UP_POS),
        READY(LIFT_HOME_POS,REACH_HOME_POS,ELBOW_READY_POS),
        LIFT_HIGH_CHAMBER(LIFT_HIGH_CHAMBER_POS,REACH_HOME_POS,ELBOW_UP_POS),
        REACH_FOR_SAMPLE(LIFT_HOME_POS,REACH_FULL_EXTEND_POS,ELBOW_AQUIRE_POS),
        AQUIRE_SAMPLE(LIFT_HOME_POS,REACH_HOME_POS,ELBOW_AQUIRE_POS),
        LIFT_HIGH_BASKET(LIFT_HIGH_BASKET_POS,REACH_HOME_POS,ELBOW_SCORE_BASKET_POS),
        LIFT_AND_REACH_HIGH_BASKET(LIFT_HIGH_BASKET_POS,REACH_FULL_EXTEND_POS,ELBOW_SCORE_BASKET_POS);
        public final int liftPos;
        public final int reachPos;
        public final double elbowPos;
        GantryStates(int liftPos, int reachPos, double  elbowPos) {
            this.liftPos = liftPos;
            this.reachPos = reachPos;
            this.elbowPos = elbowPos;
        }
    }
}
