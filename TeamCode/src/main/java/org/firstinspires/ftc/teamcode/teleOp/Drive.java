package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.backup;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorPulseT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collector_multiplierB;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collector_multiplierD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collector_multiplierN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.dMax;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.dMid;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.errorMax;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.errorMid;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.errorMin;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.pow;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.offset;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rumblingT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.tolerance;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.vMax;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.BasketLauncher;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.teamcode.util.ThankYouVV.BetterGamepad;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends OpMode {
    public Hardware robot;
    public Follower follower;
    public Pose startingPose;
    public BetterGamepad betterGamepad;
    public Timer rumbling, rumbling2, block;
    public AprilTagDetection id;
    public BasketLauncher launcher;
    public double shooterVelocity, error, distance;
    public boolean direction, team;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        rumbling = new Timer();
        rumbling2 = new Timer();
        block = new Timer();

        betterGamepad = new BetterGamepad(gamepad1);

        launcher = new BasketLauncher();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        robot.update();
        betterGamepad.update();

        drive(gamepad1);

        if (id != null) {
            distance = id.ftcPose.y-.14;
            if(distance < dMid) error = errorMin;
            else if(distance >= dMid && distance < dMax) error = errorMid;
            else error = errorMax;
            shooterVelocity = Math.min(1, launcher.multiplier(distance)+error);
            telemetry.addLine("I see!");
            telemetry.addData("distance", distance);
        }

        telemetry.addData("velocity", shooterVelocity*vMax);

        if (betterGamepad.right_trigger.pressed) direction = !direction;
        if (betterGamepad.cross.pressed) team = !team;

        if (betterGamepad.dpad_up.pressed && backup < 1.0) backup += 0.05;
        else if (betterGamepad.dpad_down.pressed && backup > 0.0) backup -= 0.05;
        telemetry.addData("backup", backup*vMax);

        if(team) {
            id = robot.getTagBySpecificID(24);
            telemetry.addLine("RED");
        } else {
            id = robot.getTagBySpecificID(20);
            telemetry.addLine("BLUE");
        }

        if (direction) {
            robot.collector.setDirection(DcMotorEx.Direction.REVERSE);
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            if (rumbling.getElapsedTime() <= rumblingT)
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
            rumbling2.resetTimer();
        } else {
            robot.collector.setDirection(DcMotorEx.Direction.FORWARD);
            gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            if (rumbling2.getElapsedTime() <= rumblingT)
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
            rumbling.resetTimer();
        }

        if (betterGamepad.right_bumper.held && id != null) {
            if (id.center.x > tolerance) robot.turn(pow, false);
            else if (id.center.x < tolerance - offset) robot.turn(pow, true);
            else {
                for (DcMotorEx chassis : robot.chassis) chassis.setPower(0);
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            }
        } else if (betterGamepad.circle.held) shoot(shooterVelocity);
        else if(betterGamepad.square.held) shoot(backup);
        else {
            for(DcMotorEx shooter : robot.shooters) shooter.setVelocity(0);
            for (Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
            if (betterGamepad.left_bumper.held) robot.collector.setVelocity(collector_multiplierD*vMax);
            else robot.collector.setVelocity(0);
            block.resetTimer();
        }

        if(betterGamepad.dpad_left.held) robot.turn(pow, true);
        else if(betterGamepad.dpad_right.held) robot.turn(pow, false);
    }

    public void drive(Gamepad gamepad){
        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                false
        );
        follower.update();
    }

    public void shoot(double velocity) {
        for(DcMotorEx shooter : robot.shooters) shooter.setVelocity(velocity*vMax);

        if(block.getElapsedTime() >= blockT) {
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerOpenPos);
            if(block.getElapsedTime() < blockT + collectorPulseT) robot.collector.setVelocity(collector_multiplierB * vMax);
            else robot.collector.setVelocity(collector_multiplierN * vMax);
        } else {
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
            robot.collector.setVelocity(0);
        }
    }
}
