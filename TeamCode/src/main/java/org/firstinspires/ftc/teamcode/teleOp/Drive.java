package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorReverse;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.distance;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.dx;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.dy;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.errorH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.errorX;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.errorY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.goalX;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.goalY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jackDownPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.jackUpPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rumblingT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.robotPose;
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
    public double angle, lastX, lastY;
    public boolean direction, shoot, jack;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(robotPose == null ? new Pose(72, 72, Math.toRadians(90)) : robotPose);
        follower.update();

        rumbling = new Timer();
        rumbling2 = new Timer();
        block = new Timer();

        if(goalX == 0) goalX = 12;

        betterGamepad = new BetterGamepad(gamepad1);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        betterGamepad.update();

        if(betterGamepad.left_trigger.pressed) direction = !direction;
        if(betterGamepad.right_trigger.pressed) jack = !jack;

        if(jack) for(Servo jack : robot.blockers) jack.setPosition(jackUpPos);
        else for(Servo jack : robot.blockers) jack.setPosition(jackDownPos);

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

        dx = goalX - follower.getPose().getX(); dy = goalY - follower.getPose().getY();
        distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        angle = Math.atan2(dy, dx);
        if(goalX == 131) angle -= Math.toRadians(angleError(distance));
        else angle += Math.toRadians(angleError(distance));
        if(angle < 0) angle += 2*Math.PI;

        if(betterGamepad.right_bumper.pressed && !shoot && ((distance >= 60 && distance <= 90) || distance >= 120)) shoot = true;
        else if(betterGamepad.right_bumper.pressed) {
            follower.startTeleopDrive(true);
            shoot = false;
        }

        if(shoot) {
            follower.holdPoint(new Pose(lastX, lastY, angle));
            if(follower.getPose().getX() >= lastX-errorX && follower.getPose().getX() <= lastX+errorX &&
                follower.getPose().getY() >= lastY-errorY && follower.getPose().getY() <= lastY+errorY &&
                follower.getPose().getHeading() >= angle-Math.toRadians(errorH) && follower.getPose().getHeading() <= angle+Math.toRadians(errorH)) shoot(shootMult(distance));
            else {
                for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(0);
                for (Servo blocker : robot.blockers) blocker.setPosition(blockerBlockedPos);
                robot.collector.setVelocity(vMax);
                block.resetTimer();
            }
        } else {
            drive(gamepad1);
            for(DcMotorEx shooter : robot.shooters) shooter.setVelocity(0);
            for (Servo blocker : robot.blockers) blocker.setPosition(blockerBlockedPos);
            if (betterGamepad.left_bumper.held) robot.collector.setVelocity(vMax);
            else robot.collector.setVelocity(0);
            lastX = follower.getPose().getX();
            lastY = follower.getPose().getY();
            block.resetTimer();
        }

        follower.update();

        telemetry.addData("robot X", follower.getPose().getX());
        telemetry.addData("robot Y", follower.getPose().getY());
        telemetry.addData("robot H", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("angle", Math.toDegrees(angle));
        telemetry.addData("distance", distance);
    }

    public void drive(Gamepad gamepad){
        if(goalX == 131) follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                false
        );
        else follower.setTeleOpDrive(
                gamepad.left_stick_y,
                gamepad.left_stick_x,
                -gamepad.right_stick_x,
                false
        );
    }

    public void shoot(double multiplier) {
        for(DcMotorEx shooter : robot.shooters) shooter.setVelocity(MathFunctions.clamp(multiplier, .01, 1) * vMax);
        for(Servo blocker : robot.blockers) blocker.setPosition(blockerOpenPos);

        if(block.getElapsedTime() >= blockT) robot.collector.setVelocity(vMax);
        else robot.collector.setPower(collectorReverse);
    }

    public double angleError(double distance) {
        return MathFunctions.clamp(4.83415*Math.pow(10, -7) * Math.pow(distance, 4) - 0.000184899 * Math.pow(distance, 3) + 0.0251689*Math.pow(distance, 2) - 1.4605*distance + 40.69425, 6, 12);
    }

    public double shootMult(double distance) {
        return MathFunctions.clamp(-3.28801*Math.pow(10, -7)*Math.pow(distance, 3) + 0.000115734*Math.pow(distance, 2) - 0.0123207*distance + 1.13939, .725, .79);
    }
}
