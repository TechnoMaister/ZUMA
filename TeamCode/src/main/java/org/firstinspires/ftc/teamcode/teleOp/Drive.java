package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleError;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorC;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorReverseMult;
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
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootErrorF;
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
    public boolean shoot, jack, tuning, reset;
    public enum ShootingZone {
        NONE,
        CLOSE,
        FAR
    }

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(robotPose == null ? new Pose(136, 8, Math.toRadians(180)) : robotPose);
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
        if(betterGamepad.touchpad.pressed) reset = !reset;

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
            if(canShoot(follower.getPose()) == ShootingZone.FAR) {

                if (betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2))) angleErrorF -= 2;
                else if (betterGamepad.dpad_left.pressed && angle > Math.toRadians(2)) angleErrorF += 2;
                angleError = angleErrorF;

                if (betterGamepad.dpad_up.pressed && (shootMult(distance) + shootError) <= .98) shootErrorF += .02;
                else if (betterGamepad.dpad_down.pressed && (shootMult(distance) + shootError) > .02) shootErrorF -= .02;
                shootError = shootErrorF;

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
        if(goalX == 132) angle -= Math.toRadians(angleError(distance)+angleError);
        else angle += Math.toRadians(angleError(distance)+angleError);
        if(angle < 0) angle += 2*Math.PI;

        if(betterGamepad.right_bumper.pressed && !shoot && canShoot(follower.getPose()) != ShootingZone.NONE && distance >= 59) shoot = true;
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

        if(reset){
            if (goalX == 132) {
                if (betterGamepad.triangle.pressed) {
                    follower.setPose(new Pose(8.66, 7.955, Math.toRadians(0)));
                    reset = false;
                }
                else if (betterGamepad.circle.pressed) {
                    follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(0)));
                    reset = false;
                }
            } else {
                if (betterGamepad.triangle.pressed) {
                    follower.setPose(new Pose(135.34, 7.955, Math.toRadians(180)));
                    reset = false;
                }
                else if (betterGamepad.circle.pressed) {
                    follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(180)));
                    reset = false;
                }
            }
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
        if(goalX == 132) follower.setTeleOpDrive(
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
        else robot.collector.setVelocity(collectorReverseMult*vMax);
    }

    public double angleError(double distance) {
        return MathFunctions.clamp(4.83415*1e-7 * Math.pow(distance, 4) - 0.000184899 * Math.pow(distance, 3) + 0.0251689*Math.pow(distance, 2) - 1.4605*distance + 40.69425, 6, 12);
    }

    public double shootMult(double distance) {
        return MathFunctions.clamp(-3.28801*1e-7 * Math.pow(distance, 3) + 0.000115734*Math.pow(distance, 2) - 0.0123207*distance + 1.13939, .725, .79);
    }

    public ShootingZone canShoot(Pose robot) {
        double width = 15.91;
        double length = 17.32;
        double xc = robot.getPose().getX();
        double yc = robot.getPose().getY();
        double heading = robot.getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double halfW = width / 2;
        double halfL = length / 2;

        Pose[] corners = new Pose[4];
        double[][] offsets = {
                {-halfW, -halfL}, {halfW, -halfL},
                {halfW, halfL}, {-halfW, halfL}
        };

        for (int i = 0; i < 4; i++) {
            double dx = offsets[i][0];
            double dy = offsets[i][1];
            double x = xc + dx * cos - dy * sin;
            double y = yc + dx * sin + dy * cos;
            corners[i] = new Pose(x, y);
        }

        boolean anyClose = false;
        boolean anyFar = false;
        for (Pose corner : corners) {
            if (isClose(corner)) anyClose = true;
            if (isFar(corner)) anyFar = true;
        }

        if (anyClose) return ShootingZone.CLOSE;
        if (anyFar) return ShootingZone.FAR;

        return ShootingZone.NONE;
    }

    private boolean isClose(Pose p) {
        double x = p.getPose().getX();
        double y = p.getPose().getY();
        return y >= x && y >= 144 - x && y <= 144;
    }

    private boolean isFar(Pose p) {
        double x = p.getPose().getX();
        double y = p.getPose().getY();
        return y >= 0 && y <= x - 49 && y <= -x + 95;
    }
}
