package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.vMax;

public class BasketLauncher {

    public double g = 9.81;
    public double deltaH = 0.72035;
    public double launchAngleDeg = 55;
    public double wheelDiameterMeters = .072, gearRatio = .5;

    public BasketLauncher() {}

    private double ballVelocity(double horizontalDistance) {
        double theta = Math.toRadians(launchAngleDeg);
        double base = horizontalDistance * Math.tan(theta) - deltaH;
        if (base <= 0) return 0;

        double numerator = g * horizontalDistance * horizontalDistance;
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * base;

        return Math.sqrt(numerator / denominator);
    }

    public double multiplier(double horizontalDistance) {
        double vBall = ballVelocity(horizontalDistance);
        double wheelCirc = Math.PI * wheelDiameterMeters;
        double wheelRPS = vBall / wheelCirc;
        double motorRPS = wheelRPS * gearRatio;

        double ticksPerSecond = motorRPS * 28.0;

        double mult = ticksPerSecond / vMax;

        if (mult < 0) mult = 0;
        if (mult > 1) mult = 1;

        return mult;
    }
}
