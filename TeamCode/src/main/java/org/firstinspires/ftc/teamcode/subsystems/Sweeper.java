package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sweeper {
    private Servo elbow;
    private Servo wheel;
    private ElapsedTime timer;
    private RevBlinkinLedDriver.BlinkinPattern lastColor;
    private NormalizedColorSensor colorSensor;
    private static final double TIME_DELAY = 0.2;
    public RevBlinkinLedDriver.BlinkinPattern colorDetected;

    public Sweeper(HardwareMap hardwareMap) {
        elbow = hardwareMap.get(Servo.class, "elbow");
        wheel = hardwareMap.get(Servo.class, "wheel");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        colorDetected = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
        timer = new ElapsedTime();
    }
    public void update() {
        parse_color();
    }
    public void setTarget(double elbow_pos, double wheel_speed) {
        elbow.setPosition(elbow_pos);
        wheel.setPosition(wheel_speed);
    }
    private void parse_color() {
        final float[] hsvValues = new float[3];

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);

        final float redMin = 10.0F;
        final float redMax = 50.0F;
        final float yellowMin = 70.0F;
        final float yellowMax = 100.0F;
        final float blueMin = 200.0F;
        final float blueMax = 250.0F;

        if(distance >= 2.3 ) { // try to ignore background colors
            colorDetected = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
            lastColor = colorDetected;
        } else if (hsvValues[0] >= redMin && hsvValues[0] <= redMax){
            if (lastColor != RevBlinkinLedDriver.BlinkinPattern.RED) {
                timer.reset();
                lastColor = RevBlinkinLedDriver.BlinkinPattern.RED;
            } else if(timer.seconds() > TIME_DELAY) {
                colorDetected = RevBlinkinLedDriver.BlinkinPattern.RED;
            }
        } else if (hsvValues[0] >= yellowMin && hsvValues[0] <= yellowMax){
            if (lastColor != RevBlinkinLedDriver.BlinkinPattern.YELLOW) {
                timer.reset();
                lastColor = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            } else if(timer.seconds() > TIME_DELAY) {
                colorDetected = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            }
        } else if (hsvValues[0] >= blueMin && hsvValues[0] <= blueMax){
            if (lastColor != RevBlinkinLedDriver.BlinkinPattern.BLUE) {
                timer.reset();
                lastColor = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            } else if(timer.seconds() > TIME_DELAY) {
                colorDetected = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            }
        } else {
            colorDetected = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
            lastColor = colorDetected;
        }

    }
    public void log(Telemetry tele) {
        tele.addData("Elbow Position: ", elbow.getPosition());
        tele.addData("Wheel Speed: ", wheel.getPosition());
        tele.addData("Color Detected: ", colorDetected);
        tele.addData("Color Distance", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
    }
}
