package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blueX;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorReverse;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.dMax;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.dMid;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.goalY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.maxMult;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.midMult;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.minMult;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.redX;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rumblingT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.slow;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.robotPose;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.team;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.vMax;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.teamcode.util.BetterGamepad;

@TeleOp(name = "Drive", group = "Drive")
public class Drive extends OpMode {
    public Hardware robot;
    public Follower follower;
    public BetterGamepad betterGamepad;
    public Timer rumbling, rumbling2, block;
    public double shootMult, speed, speedR, targetHeading, headingError, rotCmd, goalX, distance;
    public boolean direction;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(robotPose);
        follower.update();

        rumbling = new Timer();
        rumbling2 = new Timer();
        block = new Timer();

        betterGamepad = new BetterGamepad(gamepad1);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        betterGamepad.update();

        if (betterGamepad.left_trigger.pressed) direction = !direction;

        if(team) goalX = redX;
        else goalX = blueX;

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

        distance = distanceToGoal(follower.getPose());
        if(distance < dMid) shootMult = minMult;
        else if(distance >= dMid && distance <= dMax) shootMult = midMult;
        else shootMult = maxMult;

        targetHeading = headingToGoal(follower.getPose());
        headingError = angleWrap(targetHeading - follower.getPose().getHeading());
        rotCmd = Math.copySign(Math.min(Math.abs(headingError) / Math.PI, 1.0), headingError);

        if(betterGamepad.right_bumper.held) {
            follower.holdPoint(new Pose(follower.getPose().getX(), follower.getPose().getY(), rotCmd));
            shoot(shootMult);
        } else {
            drive(gamepad1);
            for(DcMotorEx shooter : robot.shooters) shooter.setVelocity(0);
            for (Servo blocker : robot.blockers) blocker.setPosition(blockerBlockedPos);
            if (betterGamepad.left_bumper.held) robot.collector.setVelocity(vMax);
            else robot.collector.setVelocity(0);
            block.resetTimer();
        }
    }

    public void drive(Gamepad gamepad){
        if(betterGamepad.left_joystick_button.held) speed = slow;
        else speed = 1;

        if(betterGamepad.right_joystick_button.held) speedR = slow;
        else speedR = 1;

        follower.setTeleOpDrive(
                -gamepad.left_stick_y*speed,
                -gamepad.left_stick_x*speed,
                -gamepad.right_stick_x*speedR,
                false
        );

        follower.update();
    }

    public void shoot(double multiplier) {
        for(DcMotorEx shooter : robot.shooters) shooter.setVelocity(MathFunctions.clamp(multiplier, .01, 1) * vMax);
        for(Servo blocker : robot.blockers) blocker.setPosition(blockerOpenPos);

        if(block.getElapsedTime() >= blockT) robot.collector.setVelocity(vMax);
        else robot.collector.setPower(collectorReverse);
    }

    public double angleWrap(double angle) { while (angle > Math.PI) angle -= 2 * Math.PI; while (angle < -Math.PI) angle += 2 * Math.PI; return angle; }

    public double headingToGoal(Pose pose) { double dx = goalX - pose.getPose().getX(); double dy = goalY - pose.getPose().getY(); return Math.atan2(dy, dx); }

    public double distanceToGoal(Pose pose) { double dx = goalX - pose.getPose().getX(); double dy = goalY - pose.getPose().getY(); return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2)); }
}
