package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FeedForwardController {
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = Double.NaN;
    private double lastTime = Double.NaN;
    private double errorSum = 0.0;
    public PIDFCoefficients coefficients;
    public double kStiction = 0.0;
    public int targetPosition = 0;
    public FeedForwardController(PIDFCoefficients coefficients) {
        this.coefficients = coefficients;
    }
    public double update(int measuredPosition) {
        double currentTime = timer.nanoseconds();
        double error = targetPosition - measuredPosition;
        if(lastTime == Double.NaN) {
            lastError = error;
            lastTime = currentTime;
            return 0.0;
        } else {
            double dt = currentTime - lastTime;
            errorSum += error * coefficients.i;// * dt;
            double errorDeriv = (error - lastError);// / dt;

            lastError = error;
            lastTime = currentTime;

            double output = error * coefficients.p +
                    errorSum +
                    errorDeriv * coefficients.d +
                    coefficients.f;
            output = (Math.abs(output) < 0.01) ? (0.0) : (output + Math.signum(output) * kStiction);
            return Math.min(Math.max(output, -1.0), 1.0);
        }
    }
}
