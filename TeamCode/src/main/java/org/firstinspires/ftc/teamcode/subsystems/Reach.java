package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Reach {
    private DcMotorEx reach_motor;
    private FeedForwardController ffc;
    public Reach(HardwareMap hardwareMap) {
        reach_motor = hardwareMap.get(DcMotorEx.class, "elevator");
        PIDFCoefficients pidf_coef = new PIDFCoefficients(0.002, 0.0, 0.0, 0.0);
        ffc = new FeedForwardController(pidf_coef);
        reach_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        reach_motor.setMotorEnable();
    }
    public void update(int targetPosition) {
        ffc.targetPosition = targetPosition;
        reach_motor.setPower(ffc.update(reach_motor.getCurrentPosition()));
    }
    public void log(Telemetry tele) {
        tele.addData("Lift current encoder ticks",
                reach_motor.getCurrentPosition());
        tele.addData("Lift motor current (A)",
                reach_motor.getCurrent(CurrentUnit.AMPS));
        tele.addData("Lift current target position",
                reach_motor.getTargetPosition());
        tele.addData("Lift motor power (+/-%FS)",
                reach_motor.getPower());
    }
}
