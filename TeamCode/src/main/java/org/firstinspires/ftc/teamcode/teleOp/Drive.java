package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleError;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorC;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorL;
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
import static org.firstinspires.ftc.teamcode.util.RobotConstants.robotPose;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rumblingT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootError;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootErrorC;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootErrorL;
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
    public boolean shoot, jack, tuning;
    public enum ShootingZone {
        NONE,
        CLOSE,
        LONG
    }

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(robotPose == null ? new Pose(72, 72, Math.toRadians(90)) : robotPose);
        follower.update();

        rumbling = new Timer();
        rumbling2 = new Timer();
        block = new Timer();

        if(goalX == 0) {
            goalX = 12;
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }

        betterGamepad = new BetterGamepad(gamepad1);
    }

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        betterGamepad.update();

        if(betterGamepad.right_trigger.pressed) jack = !jack;
        if(betterGamepad.cross.pressed) tuning = !tuning;

        if(jack) for(Servo jack : robot.blockers) jack.setPosition(jackUpPos);
        else for(Servo jack : robot.blockers) jack.setPosition(jackDownPos);

        if (tuning) {
            if (rumbling.getElapsedTime() <= rumblingT)
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
            rumbling2.resetTimer();
        } else {
            if (rumbling2.getElapsedTime() <= rumblingT)
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
            rumbling.resetTimer();
        }

        if(tuning && shoot){
            if(canShoot(follower.getPose()) == ShootingZone.LONG) {

                if (betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2))) angleErrorL -= 2;
                else if (betterGamepad.dpad_left.pressed && angle > Math.toRadians(2)) angleErrorL += 2;
                angleError = angleErrorL;

                if (betterGamepad.dpad_up.pressed && (shootMult(distance) + shootError) <= .98) shootErrorL += .02;
                else if (betterGamepad.dpad_down.pressed && (shootMult(distance) + shootError) > .02) shootErrorL -= .02;
                shootError = shootErrorL;

            } else {

                if(betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2))) angleErrorC -= 2;
                else if(betterGamepad.dpad_left.pressed && angle > Math.toRadians(2)) angleErrorC += 2;
                angleError = angleErrorC;

                if(betterGamepad.dpad_up.pressed && (shootMult(distance) + shootError) <= .98) shootErrorC += .02;
                else if(betterGamepad.dpad_down.pressed && (shootMult(distance) + shootError) > .02) shootErrorC -= .02;
                shootError = shootErrorC;
            }
        }

        dx = goalX - follower.getPose().getX(); dy = goalY - follower.getPose().getY();
        distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        angle = Math.atan2(dy, dx);
        if(goalX == 131) {
            angle -= Math.toRadians(angleError(distance));
            angle -= Math.toRadians(angleError);
        } else {
            angle += Math.toRadians(angleError(distance));
            angle += Math.toRadians(angleError);
        }
        if(angle < 0) angle += 2*Math.PI;

        if(betterGamepad.right_bumper.pressed && !shoot && canShoot(follower.getPose()) != ShootingZone.NONE && distance >= 60) shoot = true;
        else if(betterGamepad.right_bumper.pressed) {
            follower.startTeleopDrive(true);
            shoot = false;
        }

        if(shoot) {
            follower.holdPoint(new Pose(lastX, lastY, angle));
            if(follower.getPose().getX() >= lastX-errorX && follower.getPose().getX() <= lastX+errorX &&
                follower.getPose().getY() >= lastY-errorY && follower.getPose().getY() <= lastY+errorY &&
                follower.getPose().getHeading() >= angle-Math.toRadians(errorH) && follower.getPose().getHeading() <= angle+Math.toRadians(errorH) && !tuning) shoot(shootMult(distance)+shootError);
            else {
                for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(0);
                for (Servo blocker : robot.blockers) blocker.setPosition(blockerBlockedPos);
                if(tuning) robot.collector.setVelocity(0);
                else robot.collector.setVelocity(vMax);
                block.resetTimer();
            }
        } else {
            drive(gamepad1);
            for(DcMotorEx shooter : robot.shooters) shooter.setVelocity(0);
            for (Servo blocker : robot.blockers) blocker.setPosition(blockerBlockedPos);
            if(betterGamepad.left_trigger.held) robot.collector.setVelocity(-gamepad1.left_trigger*vMax);
            else if(betterGamepad.left_bumper.held) robot.collector.setVelocity(vMax);
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
        telemetry.addData("distance", distance + " IN");
        telemetry.addData("shoot multiplier", shootMult(distance)+shootError);
        telemetry.addData("zone", canShoot(follower.getPose()));
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
        else robot.collector.setVelocity(collectorReverse*vMax);
    }

    public double angleError(double distance) {
        return MathFunctions.clamp(4.83415*1e-7 * Math.pow(distance, 4) - 0.000184899 * Math.pow(distance, 3) + 0.0251689*Math.pow(distance, 2) - 1.4605*distance + 40.69425, 6, 12);
    }

    public double shootMult(double distance) {
        return MathFunctions.clamp(-3.28801*1e-7 * Math.pow(distance, 3) + 0.000115734*Math.pow(distance, 2) - 0.0123207*distance + 1.13939, .725, .79);
    }

    public ShootingZone canShoot(Pose robot) {

        double halfL = 3.94, halfl = 3.54, xc = robot.getPose().getX(), yc = robot.getPose().getY(), eps = 1e-6;

        Pose[] corners = {
                new Pose(xc - halfL, yc - halfl),
                new Pose(xc + halfL, yc - halfl),
                new Pose(xc + halfL, yc + halfl),
                new Pose(xc - halfL, yc + halfl)
        };

        boolean inTop = false, inBottom = false;

        for (Pose corner : corners) {
            double x = corner.getPose().getX(), y = corner.getPose().getY();

            if (y >= x - eps && y >= 144 - x - eps && y <= 144 + eps) inTop = true;

            if (y >= 0 - eps && y <= x - 49 + eps && y <= -x + 95 + eps) inBottom = true;
        }

        if (!inTop)
            if (yc >= xc - eps && yc >= 144 - xc - eps && yc <= 144 + eps) inTop = true;

        if (!inBottom)
            if (yc >= 0 - eps && yc <= xc - 49 + eps && yc <= -xc + 95 + eps) inBottom = true;

        if (inTop) return ShootingZone.CLOSE;
        if (inBottom) return ShootingZone.LONG;
        return ShootingZone.NONE;
    }
}
