package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorWithEncoderAndController {
    private int tolerance;
    private DcMotorEx motor;
    private FeedForwardController ffc;
    private ElapsedTime timer = new ElapsedTime();
    private final String name;
    private boolean isFlipped;
    private boolean isHoming;
    private boolean isHomed;
    private boolean isOverdrive;
    public MotorWithEncoderAndController(HardwareMap hardwareMap, Config config) {
        name = config.deviceName;
        motor = hardwareMap.get(DcMotorEx.class, config.deviceName);
        ffc = new FeedForwardController(config.coefficients);
        ffc.kStiction = config.kStiction;
        motor.setDirection(config.direction);
        motor.setMotorEnable();
        tolerance = config.tolerance;
        isFlipped = config.isFlipped;
        isHoming = config.isHoming;
        isHomed = false;
        isOverdrive = false;
    }
    public void update() {
        int pos = motor.getCurrentPosition();
        int targetPosition = ffc.targetPosition;
        double motorPower = ffc.update(pos);

        if(isHoming) {
            if (!isHomed && (targetPosition == 0) && (Math.abs(pos - targetPosition) < 100)) {
                if (!isOverdrive) {
                    timer.reset();
                    isOverdrive = true;
                    motorPower = 0.2;
                } else if (timer.seconds() < 0.5) {
                    motorPower = 0.2;
                } else {
                    motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                    isHomed = true;
                    isOverdrive = false;
                    motorPower = 0.0;
                }
            }
        }

        if (isFlipped){
            motor.setPower(-motorPower);
        } else {
            motor.setPower(motorPower);
        }
    }
    public void setTarget(int targetPosition) {
        ffc.targetPosition = targetPosition;
        if(targetPosition != 0) {
            isOverdrive = false;
            isHomed = false;
        }
    }
    public boolean isDone() {
        int error = motor.getCurrentPosition() - ffc.targetPosition;
        boolean temp = Math.abs(error) < tolerance;
        if (temp){
            motor.setPower(-ffc.coefficients.f);
        }
        return temp;
    }
    public void log(Telemetry tele) {
        tele.addData(name + " current encoder ticks",   motor.getCurrentPosition());
        tele.addData(name + " motor current (A)",       motor.getCurrent(CurrentUnit.AMPS));
        tele.addData(name + " current target position", ffc.targetPosition);
        tele.addData(name + " motor power (+/-%FS)",    motor.getPower());
//        tele.addData(name + " daryl's timer",           timer.seconds());
//        tele.addData(name + " motor mode", motor.getMode());
    }

    public void autonLog(TelemetryPacket packet) {
        packet.put(name + " current encoder ticks",   motor.getCurrentPosition());
        packet.put(name + " current target position", ffc.targetPosition);
        packet.put(name + " motor power (+/-%FS)",    motor.getPower());
    }
    public static class Config {
        public String deviceName;
        public PIDFCoefficients coefficients;
        public double kStiction;
        public DcMotorSimple.Direction direction;
        public boolean isFlipped;
        public int tolerance;
        public boolean isHoming;
        public Config(String deviceName,
                      PIDFCoefficients coefficients,
                      double kStiction,
                      DcMotorSimple.Direction direction, int tolerance, boolean isFlipped, boolean isHoming) {
            this.deviceName = deviceName;
            this.coefficients = coefficients;
            this.kStiction = kStiction;
            this.direction = direction;
            this.tolerance = tolerance;
            this.isFlipped = isFlipped;
            this.isHoming = isHoming;
        }
    }
}
