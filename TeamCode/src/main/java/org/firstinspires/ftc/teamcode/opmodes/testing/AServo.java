package org.firstinspires.ftc.teamcode.opmodes.testing;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AServo {
    private Servo aServo;

    public AServo(HardwareMap hardwareMap) {
        aServo = hardwareMap.get(Servo.class, "a_servo");
    }

    public Action OpenServo() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    aServo.setPosition(1);
                    initialized = true;
                }

                double pos = aServo.getPosition();
                packet.put("aServo position", pos);
                return false;
            }
        };
    }

    public Action CloseServo() {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    aServo.setPosition(0.25);
                    initialized = true;
                }

                double pos = aServo.getPosition();
                packet.put("aServo position", pos);
                return false;
            }
        };
    }
}
