package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sweeper {
    private Servo elbow;
    private Servo wheel;
    private NormalizedColorSensor colorSensor;
    public RevBlinkinLedDriver.BlinkinPattern colorDetected;

    public Sweeper(HardwareMap hardwareMap) {
        elbow = hardwareMap.get(Servo.class, "elbow");
        wheel = hardwareMap.get(Servo.class, "wheel");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");
        colorDetected = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
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

        if(distance >= 2.0 ) { // try to ignore background colors
            colorDetected = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
        } else if (hsvValues[0] >= redMin && hsvValues[0] <= redMax){
            colorDetected = RevBlinkinLedDriver.BlinkinPattern.RED;
        } else if (hsvValues[0] >= yellowMin && hsvValues[0] <= yellowMax){
            colorDetected = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        } else if (hsvValues[0] >= blueMin && hsvValues[0] <= blueMax){
            colorDetected = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        } else {
            colorDetected = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
        }
    }
    public void log(Telemetry tele) {
        tele.addData("Elbow Position: ", elbow.getPosition());
        tele.addData("Wheel Speed: ", wheel.getPosition());
        tele.addData("Color Detected: ", colorDetected);
    }
}
