package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.vMax;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class BasketLauncher {
    public double
    g = 9.81,
    NOMINAL_VOLTAGE = 12,
    deltaH = .72035,
    launchAngleDeg = 55;

    public BasketLauncher(){}

    public double power(double horizontalDistance, HardwareMap hardwareMap) {
        double theta = Math.toRadians(launchAngleDeg);
        double base = horizontalDistance * Math.tan(theta) - deltaH;
        if (base <= 0) return 0;
        double numerator = g * horizontalDistance * horizontalDistance;
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * base;
        double v = Math.sqrt(numerator / denominator);

        double batteryVoltage = Double.MAX_VALUE;
        for (VoltageSensor vs : hardwareMap.voltageSensor)
            batteryVoltage = Math.min(batteryVoltage, vs.getVoltage());

        double motorPower = (v / vMax) * (NOMINAL_VOLTAGE / batteryVoltage);

        return Math.max(0, Math.min(1, motorPower));
    }
}