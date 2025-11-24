package org.firstinspires.ftc.teamcode.util;

public class BasketLauncher {

    public double g = 9.81;
    public double deltaH = 0.72035;
    public double launchAngleDeg = 55;

    public BasketLauncher() {}

    public double computeRequiredVelocity(double horizontalDistance) {

        double theta = Math.toRadians(launchAngleDeg);

        double base = horizontalDistance * Math.tan(theta) - deltaH;
        if (base <= 0) return 0;

        double numerator = g * horizontalDistance * horizontalDistance;
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * base;

        return Math.sqrt(numerator / denominator);
    }
}
