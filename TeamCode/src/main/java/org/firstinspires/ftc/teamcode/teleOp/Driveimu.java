package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.backup;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorPowerB;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorPowerD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorPowerN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.k;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rumblingT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.tolerance;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.ThankYouVV.PDFS;
import org.firstinspires.ftc.teamcode.util.ThankYouVV.DT_Utils;
import org.firstinspires.ftc.teamcode.util.BasketLauncher;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Driveimu", group = "TeleOp")
public class Driveimu extends OpMode {
    public Hardware robot;
    public static IMU imu;
    public Follower follower;
    public Pose startingPose;
    public Gamepad previousGamepad1, currentGamepad1;
    public Timer rumbling, rumbling2, block;
    public boolean direction, team;
    public AprilTagDetection id;
    public BasketLauncher speed;
    public double shooterPeed;

    private double heading, relative_heading, start_heading, locked_in_heading;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
        imu=hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters=new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT


        ));
        imu.initialize(parameters);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        rumbling = new Timer();
        rumbling2 = new Timer();
        block = new Timer();

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

        speed = new BasketLauncher();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        robot.update();

        if (id != null) {
            shooterPeed = speed.speed(id.ftcPose.y);
            gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            telemetry.addLine("I see!");
        }

        telemetry.addData("shooterPower", shooterPeed);
        telemetry.addData("backup", backup);
        if(id!=null)telemetry.addData("pos",id.center.x);

        //drive(gamepad1);

        YawPitchRollAngles orientation=imu.getRobotYawPitchRollAngles();
        heading=orientation.getYaw(AngleUnit.RADIANS);
        relative_heading=AngleUnit.normalizeRadians(start_heading-orientation.getYaw(AngleUnit.RADIANS));

        if(gamepad1.circle)
        {
            start_heading=heading;
            locked_in_heading=0;
        }

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.right_trigger > 0 && previousGamepad1.right_trigger == 0) direction = !direction;
        if (currentGamepad1.cross && !previousGamepad1.cross) team = !team;

        if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up && backup <= 1) backup += .05;
        else if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down && backup >= 0) backup -= .05;

        if(team) {
            id = robot.getTagBySpecificID(24);
            telemetry.addLine("RED");
        } else {
            id = robot.getTagBySpecificID(20);
            telemetry.addLine("BLUE");
        }

        if (direction) {
            robot.collector.setDirection(DcMotorSimple.Direction.REVERSE);
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            if (rumbling.getElapsedTime() <= rumblingT)
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
            rumbling2.resetTimer();
        } else {
            robot.collector.setDirection(DcMotorSimple.Direction.FORWARD);
            gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            if (rumbling2.getElapsedTime() <= rumblingT)
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
            rumbling.resetTimer();
        }

        if (gamepad1.left_bumper) robot.collector.setVelocity(collectorPowerD);
        else if (!gamepad1.right_bumper) robot.collector.setVelocity(0);

        if (gamepad1.right_bumper && id != null) {
            if(id.center.x >= tolerance &&
                    id.center.x <= tolerance)
            shoot(backup);
            else if(id.center.x < tolerance) {
                robot.leftFront.setVelocity(-k);
                robot.leftRear.setVelocity(-k);
                robot.rightFront.setVelocity(k);
                robot.rightRear.setVelocity(k);
            } else {
                robot.leftFront.setVelocity(k);
                robot.leftRear.setVelocity(k);
                robot.rightFront.setVelocity(-k);
                robot.rightRear.setVelocity(-k);
            }
        } else if(gamepad1.circle) shoot(backup);
        else {
            for (DcMotorEx shooterMotor : robot.shooters) shooterMotor.setVelocity(0);
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
            block.resetTimer();
        }
    }


    public static class heading_manager {
        public static double P=0.42,D=0.000001,F=0.03,DELTA = 0.10;
        public static PDFS pdfs = new PDFS();
        public static double update(double delta) {
            pdfs.set_coeffs(P,D,0,0);
            pdfs.set_error_delta(DELTA);
            double output = pdfs.update(0, Math.abs(delta)) * Math.signum(delta);
            return output + Math.signum(output)*F;
        }
    }



    public void handle_wheels() {

        double power_rotate = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));
        double angle_rotate = Math.atan2(gamepad1.right_stick_x, -gamepad1.right_stick_y);
        double rx, target_angle;

        double x,y,rotX,rotY;

        target_angle = locked_in_heading;
        rx = heading_manager.update(DT_Utils.calc_turn_angle(relative_heading, target_angle));
        if (power_rotate > 0.05) {
            locked_in_heading = relative_heading;
            target_angle = angle_rotate;
            rx = heading_manager.update(DT_Utils.calc_turn_angle(relative_heading, target_angle)) * power_rotate;
        }
        y = gamepad1.right_stick_y;
        x = gamepad1.left_stick_x;
            rotX = x * Math.cos(relative_heading) - y * Math.sin(relative_heading);
            rotY = x * Math.sin(relative_heading) + y * Math.cos(relative_heading);


            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            robot.leftFront.setVelocity(frontLeftPower);
            robot.leftRear.setVelocity(backLeftPower);
            robot.rightFront.setVelocity(frontRightPower);
                robot.rightRear.setVelocity(backRightPower);

    }

    /*public void drive(Gamepad gamepad){
        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x
        );
        follower.update();
    }

     */

    public void shoot(double speed) {
        for (DcMotorEx shooterMotor : robot.shooters) shooterMotor.setVelocity(speed);
        if(block.getElapsedTime() >= blockT) {
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerOpenPos);
            robot.collector.setVelocity(collectorPowerN);
        }
        else {
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
            robot.collector.setVelocity(collectorPowerB);
        }
    }
}