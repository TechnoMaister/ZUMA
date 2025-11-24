package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PDFS {
    public ElapsedTime timer = new ElapsedTime();
    private double P,D,F,S,MIN_POS,MAX_POS = 1,ERROR_DELTA,last_error;
    private boolean use_sqrt = false;

    public double update(double pos, double destination) {
        double error = destination - pos;
        double derivative = (error - this.last_error) / timer.seconds();
        timer.reset();
        this.last_error = error;
        if (Math.abs(error) < ERROR_DELTA) error = 0;
        double output = (use_sqrt ? Math.sqrt(P * error) : (P * error)) + D * derivative + F * (pos - MIN_POS)/(MAX_POS - MIN_POS) + (error > ERROR_DELTA ? S : 0);

        return Math.min(1, Math.max(-1, output));
    }

    public void set_min_pos(double pos) {
        this.MIN_POS = pos;
    }

    public void set_max_pos(double pos) {
        this.MAX_POS = pos;
    }

    public void set_error_delta(double error_delta) {
        this.ERROR_DELTA = error_delta;
    }

    public void set_coeffs(double Kp, double Kd, double Kf, double Ks) {
        this.P = Kp;
        this.D = Kd;
        this.F = Kf;
        this.S = Ks;
    }

    public void set_sqrt(boolean enabled) {
        this.use_sqrt = enabled;
    }
}