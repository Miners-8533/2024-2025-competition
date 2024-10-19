package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Lift {
    private DcMotorEx lift_motor;

    public Lift(HardwareMap hardwareMap) {
        lift_motor = hardwareMap.get(DcMotorEx.class, "elevator");

        PIDFCoefficients pidf_coef = new PIDFCoefficients(0.002, 0.0, 0.0, 0.02);
        lift_motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf_coef);

        lift_motor.setMotorEnable();//TODO test removing this
    }

    public void update(int target_position) {
        lift_motor.setTargetPosition(target_position);
    }

    public void log(Telemetry tele) {
        tele.addData("Lift current encoder ticks",
                lift_motor.getCurrentPosition());
        tele.addData("Lift motor current (A)",
                lift_motor.getCurrent(CurrentUnit.AMPS));
        tele.addData("Lift current target position",
                lift_motor.getTargetPosition());
        tele.addData("Lift motor power (+/-%FS)",
                lift_motor.getPower());
    }

}
