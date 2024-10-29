package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorWithEncoderAndController {
    private DcMotorEx motor;
    private FeedForwardController ffc;
    private final String name;
    public MotorWithEncoderAndController(HardwareMap hardwareMap, Config config) {
        name = config.deviceName;
        motor = hardwareMap.get(DcMotorEx.class, config.deviceName);
        ffc = new FeedForwardController(config.coefficients);
        ffc.kStiction = config.kStiction;
        motor.setDirection(config.direction);
        motor.setMotorEnable();
    }
    public void update(int targetPosition) {
        ffc.targetPosition = targetPosition;
        motor.setPower(ffc.update(motor.getCurrentPosition()));
    }
    public boolean isDone() {
        int error = motor.getCurrentPosition() - ffc.targetPosition;
        return Math.abs(error) < SubSystemConfigs.FFC_TOLERANCE;
    }
    public void log(Telemetry tele) {
        tele.addData(name + "current encoder ticks",   motor.getCurrentPosition());
        tele.addData(name + "motor current (A)",       motor.getCurrent(CurrentUnit.AMPS));
        tele.addData(name + "current target position", ffc.targetPosition);
        tele.addData(name + "motor power (+/-%FS)",    motor.getPower());
    }
    public static class Config {
        public String deviceName;
        public PIDFCoefficients coefficients;
        public double kStiction;
        public DcMotorSimple.Direction direction;
        public Config(String deviceName,
                      PIDFCoefficients coefficients,
                      double kStiction,
                      DcMotorSimple.Direction direction) {
            this.deviceName = deviceName;
            this.coefficients = coefficients;
            this.kStiction = kStiction;
            this.direction = direction;
        }
    }
}
