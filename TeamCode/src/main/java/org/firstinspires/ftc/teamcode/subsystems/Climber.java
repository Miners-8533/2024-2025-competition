package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Climber {

    private DcMotorEx climb_motor;

    public Climber(HardwareMap hardwareMap) {
        climb_motor = hardwareMap.get(DcMotorEx.class, "climb");

        PIDFCoefficients pidf_coef = new PIDFCoefficients(0.002, 0.0, 0.0, 0.0);
        climb_motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf_coef);

        climb_motor.setMotorEnable();//TODO test removing this
    }

    public void update(int target_position) { climb_motor.setTargetPosition(target_position); }

    public void log(Telemetry tele) {
        tele.addData("Climb current encoder ticks",
                climb_motor.getCurrentPosition());
        tele.addData("Climb motor current (A)",
                climb_motor.getCurrent(CurrentUnit.AMPS));
        tele.addData("Climb current target position",
                climb_motor.getTargetPosition());
        tele.addData("Climb motor power (+/-%FS)",
                climb_motor.getPower());
    }

}
