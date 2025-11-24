package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MyIMU", group = "IMU")
public class MyIMU extends OpMode {

    public static com.qualcomm.robotcore.hardware.IMU imu;
    public Hardware robot;
    public static BetterGamepad betterGamepad;
    public static DcMotorEx leftFront, leftRear, rightFront, rightRear;
    public static double heading, relative_heading, start_heading, locked_in_heading;

    @Override
    public void init() {
        betterGamepad =new BetterGamepad(gamepad1);
        robot = new Hardware(hardwareMap);
    }

    @Override
    public void loop() {
        betterGamepad.update();
        YawPitchRollAngles orientation=imu.getRobotYawPitchRollAngles();

        heading=orientation.getYaw(AngleUnit.RADIANS);
        relative_heading=AngleUnit.normalizeRadians(start_heading-orientation.getYaw(AngleUnit.RADIANS));

        if(gamepad1.circle) {
            start_heading=heading;
            locked_in_heading=0;
        }
        handle_wheels();
    }

    public static class heading_manager {
        public static double P=0.2,D=0.00000,F=0.0,DELTA = 0.10;
        public static PDFS pdfs = new PDFS();
        public static double update(double delta) {
            pdfs.set_coeffs(P,D,0,0);
            pdfs.set_error_delta(DELTA);
            double output = pdfs.update(0, Math.abs(delta)) * Math.signum(delta);
            return output + Math.signum(output)*F;
        }
    }

    public void handle_wheels() {

        double power_rotate = Math.sqrt(Math.pow(betterGamepad.right_joystick.x, 2) + Math.pow(betterGamepad.right_joystick.y, 2));
        double angle_rotate = Math.atan2(betterGamepad.right_joystick.x, -betterGamepad.right_joystick.y);
        double rx, target_angle;

        double x,y,rotX,rotY;

        target_angle = locked_in_heading;
        rx = heading_manager.update(DT_Utils.calc_turn_angle(relative_heading, target_angle));
        if (power_rotate > 0.05) {
            locked_in_heading = relative_heading;
            target_angle = angle_rotate;
            rx = heading_manager.update(DT_Utils.calc_turn_angle(relative_heading, target_angle)) * power_rotate;
        }
        y = -betterGamepad.left_joystick.y;
        x = betterGamepad.left_joystick.x;
        rotX = x * Math.cos(relative_heading) - y * Math.sin(relative_heading);
        rotY = x * Math.sin(relative_heading) + y * Math.cos(relative_heading);


        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

    }
}
