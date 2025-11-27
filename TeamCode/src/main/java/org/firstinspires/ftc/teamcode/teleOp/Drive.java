package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.RPM435MAX;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.backup;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorSpeedB;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorSpeedD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorSpeedN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.k;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rumblingT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.tolerance;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.BasketLauncher;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends OpMode {
    public Hardware robot;
    public Follower follower;
    public Pose startingPose;
    public Gamepad previousGamepad1, currentGamepad1;
    public Timer rumbling, rumbling2, block;
    public boolean direction, team;
    public AprilTagDetection id;
    public BasketLauncher speed;
    public double shooterSpeed;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

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
            shooterSpeed = speed.computeRequiredVelocity(id.ftcPose.y);
            gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            telemetry.addLine("I see!");
        }

        telemetry.addData("shooterPower", shooterSpeed);
        telemetry.addData("backup", backup);
        if(id!=null)telemetry.addData("pos",id.center.x);

        drive(gamepad1);

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

        if (gamepad1.left_bumper) robot.collector.setVelocity(collectorSpeedD*RPM435MAX);
        else if (!gamepad1.right_bumper) robot.collector.setVelocity(0);

        if (gamepad1.right_bumper && id != null) {
            if (id.center.x > tolerance) {
                robot.leftFront.setPower(k);
                robot.leftRear.setPower(k);
                robot.rightFront.setPower(-k);
                robot.rightRear.setPower(-k);
            } else if (id.center.x < -tolerance) {
                robot.leftFront.setPower(-k);
                robot.leftRear.setPower(-k);
                robot.rightFront.setPower(k);
                robot.rightRear.setPower(k);
            } else {
                shoot(backup * RPM435MAX);
                robot.leftFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightRear.setPower(0);
            }
        } else if(gamepad1.circle) shoot(backup*RPM435MAX);
        else {
            for (DcMotorEx shooterMotor : robot.shooters) shooterMotor.setVelocity(0);
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
            block.resetTimer();
        }
    }

    public void drive(Gamepad gamepad){
        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x
        );
        follower.update();
    }

    public void shoot(double speed) {
        for (DcMotorEx shooterMotor : robot.shooters) shooterMotor.setVelocity(speed);
        if(block.getElapsedTime() >= blockT) {
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerOpenPos);
            robot.collector.setVelocity(collectorSpeedN*RPM435MAX);
        }
        else {
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
            robot.collector.setVelocity(collectorSpeedB*RPM435MAX);
        }
    }
}
