package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name="Color Sensor V3", group="Testing")
public class ConceptColorSensor extends LinearOpMode {

    @Override
    public void runOpMode() {
        final float[] hsvValues = new float[3];

        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            float redMin = 10.0F;
            float redMax = 50.0F;
            float yellowMin = 70.0F;
            float yellowMax = 100.0F;
            float blueMin = 200.0F;
            float blueMax = 250.0F;

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            if (hsvValues[0] >= redMin && hsvValues[0] <= redMax){
                telemetry.addData("result", "found red");
            }
            else if (hsvValues[0] >= yellowMin && hsvValues[0] <= yellowMax){
                telemetry.addData("result",  "found yellow");
            }
            else if (hsvValues[0] >= blueMin && hsvValues[0] <= blueMax){
                telemetry.addData("result",  "found Blue");
            }
            else {
                telemetry.addData("result", "no match");
            }
            /* If this color sensor also has a distance sensor, display the measured distance.
             * Note that the reported distance is only useful at very close range, and is impacted by
             * ambient light and surface reflectivity. */
            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            telemetry.update();
        }
    }
}

