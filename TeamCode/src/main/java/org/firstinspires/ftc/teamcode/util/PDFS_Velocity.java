package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PDFS_Velocity {

    private double Kp, Kd, Kf;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public PDFS_Velocity(double Kp, double Kd, double Kf) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Kf = Kf;
        timer.reset();
    }

    public double update(double velocity, double targetVelocity) {

        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) dt = 0.02;

        double error = targetVelocity - velocity;

        double derivative = (error - lastError) / dt;
        lastError = error;

        double ff = Kf * targetVelocity;

        double output = Kp * error + Kd * derivative + ff;

        return Math.min(1, Math.max(0, output));
    }

    public void setCoeffs(double p, double d, double f) {
        this.Kp = p;
        this.Kd = d;
        this.Kf = f;
    }
}
