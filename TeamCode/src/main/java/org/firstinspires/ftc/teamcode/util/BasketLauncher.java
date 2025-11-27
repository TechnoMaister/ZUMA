package org.firstinspires.ftc.teamcode.util;

public class BasketLauncher {

    public double g = 9.81;
    public double deltaH = 0.72035;
    public double launchAngleDeg = 55;
    private final double wheelDiameter = 0.104;
    private final double ticksPerRev = 28;
    private final double gearRatio = 1.0;

    public BasketLauncher() {}

    public double speed(double horizontalDistance) {

        double theta = Math.toRadians(launchAngleDeg);
        double base = horizontalDistance * Math.tan(theta) - deltaH;

        if (base <= 0) return 0;

        double numerator = g * horizontalDistance * horizontalDistance;
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * base;

        double circumference = Math.PI * wheelDiameter;

        double wheelRPS = Math.sqrt(numerator / denominator) / circumference;

        double motorRPS = wheelRPS * gearRatio;

        return motorRPS * ticksPerRev;
    }
}