package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleError;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorBCL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorBCR;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorBFL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorBFR;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorRCL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorRCR;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorRFL;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.angleErrorRFR;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorReverseMult;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.delayT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.distance;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.dx;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.dy;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.errorH;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.errorX;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.errorY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.goalX;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.goalY;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.leftJackDownPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.leftJackUpPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rightJackDownPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rightJackUpPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.robotPose;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootError;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootErrorC;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shootErrorF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.startPoseF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.vMax;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.limelightOffsetX;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.limelightAngleOffsetC;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.limelightAngleOffsetF;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.referenceVoltage;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.preShootReverseT;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
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
    public Timer block, delay;
    public double angle, lastX, lastY;
    public boolean shoot, jack, tuning, r, limelightAim;

    public enum ShootingZone {
        NONE,
        CLOSE,
        FAR
    }

    Pose start;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

        start = new Pose(135.34, 7.955, Math.toRadians(180));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(robotPose == null ? start : robotPose);
        follower.update();

        block = new Timer();
        delay = new Timer();

        if (goalX == 0)
            goalX = 12; // 132

        if (goalX == 12)
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        else
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);

        betterGamepad = new BetterGamepad(gamepad1);
    }

    @Override
    public void start() {

        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        betterGamepad.update();

        if (betterGamepad.right_trigger.pressed)
            jack = !jack;
        if (betterGamepad.cross.pressed)
            tuning = !tuning;
        if (betterGamepad.square.pressed)
            limelightAim = !limelightAim;
        if (betterGamepad.circle.pressed) {
            if (goalX == 12) {
                goalX = 132;
                gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            } else {
                goalX = 12;
                gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            }
        }

        if (jack) {
            robot.leftJack.setPosition(leftJackUpPos);
            robot.rightJack.setPosition(rightJackUpPos);
        } else {
            robot.leftJack.setPosition(leftJackDownPos);
            ;
            robot.rightJack.setPosition(rightJackDownPos);
        }

        if (canShoot(follower.getPose()) == ShootingZone.FAR) {
            if (betterGamepad.dpad_up.pressed)
                shootErrorF += .02;
            else if (betterGamepad.dpad_down.pressed)
                shootErrorF -= .02;
            shootError = shootErrorF;
        } else if (canShoot(follower.getPose()) == ShootingZone.CLOSE) {
            if (betterGamepad.dpad_up.pressed)
                shootErrorC += .02;
            else if (betterGamepad.dpad_down.pressed)
                shootErrorC -= .02;
            shootError = shootErrorC;
        }

        if (goalX == 12) {

            if (canShoot(follower.getPose()) == ShootingZone.FAR) {

                if (follower.getPose().getHeading() > 90) {
                    if (betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2)))
                        angleErrorBFR -= 2;
                    else if (betterGamepad.dpad_left.pressed && angle > Math.toRadians(2))
                        angleErrorBFR += 2;
                    angleError = angleErrorBFR;
                } else {
                    if (betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2)))
                        angleErrorBFL -= 2;
                    else if (betterGamepad.dpad_left.pressed && angle > Math.toRadians(2))
                        angleErrorBFL += 2;
                    angleError = angleErrorBFL;
                }

            } else {

                if (follower.getPose().getHeading() > 90) {
                    if (betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2)))
                        angleErrorBCR -= 2;
                    else if (betterGamepad.dpad_left.pressed && angle > Math.toRadians(2))
                        angleErrorBCR += 2;
                    angleError = angleErrorBCR;
                } else {
                    if (betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2)))
                        angleErrorBCL -= 2;
                    else if (betterGamepad.dpad_left.pressed && angle > Math.toRadians(2))
                        angleErrorBCL += 2;
                    angleError = angleErrorBCL;
                }
            }

        } else {

            if (canShoot(follower.getPose()) == ShootingZone.FAR) {

                if (follower.getPose().getHeading() > 90) {
                    if (betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2)))
                        angleErrorRFR += 2;
                    else if (betterGamepad.dpad_left.pressed && angle > Math.toRadians(2))
                        angleErrorRFR -= 2;
                    angleError = angleErrorRFR;
                } else {
                    if (betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2)))
                        angleErrorRFL += 2;
                    else if (betterGamepad.dpad_left.pressed && angle > Math.toRadians(2))
                        angleErrorRFL -= 2;
                    angleError = angleErrorRFL;
                }

            } else {

                if (follower.getPose().getHeading() > 90) {
                    if (betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2)))
                        angleErrorRCR += 2;
                    else if (betterGamepad.dpad_left.pressed && angle > Math.toRadians(2))
                        angleErrorRCR -= 2;
                    angleError = angleErrorRCR;
                } else {
                    if (betterGamepad.dpad_right.pressed && angle <= (2 * Math.PI - Math.toRadians(2)))
                        angleErrorRCL += 2;
                    else if (betterGamepad.dpad_left.pressed && angle > Math.toRadians(2))
                        angleErrorRCL -= 2;
                    angleError = angleErrorRCL;
                }
            }
        }

        dx = goalX - follower.getPose().getX();
        dy = goalY - follower.getPose().getY();
        distance = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        int targetTagId = (goalX == 132) ? 24 : 20;
        int targetPipeline = (goalX == 132) ? 1 : 0;
        robot.limelight.setPipeline(targetPipeline);

        LLResultTypes.FiducialResult fiducial = robot.limelight.getFiducialById(targetTagId);

        if (!shoot) {
            if (limelightAim && fiducial != null) {
                double tx = fiducial.getTargetXDegrees();

                double distanceCm = distance * 2.54;

                double offsetAngleDegrees = Math.toDegrees(Math.atan(limelightOffsetX / distanceCm));

                double currentAngleOffset = 0;
                if (canShoot(follower.getPose()) == ShootingZone.CLOSE) {
                    currentAngleOffset = limelightAngleOffsetC;
                } else if (canShoot(follower.getPose()) == ShootingZone.FAR) {
                    currentAngleOffset = limelightAngleOffsetF;
                }

                double targetTx = offsetAngleDegrees + currentAngleOffset;

                angle = follower.getPose().getHeading() - Math.toRadians(tx - targetTx);
            } else {
                angle = Math.atan2(dy, dx);
                if (goalX == 132)
                    angle -= Math.toRadians(angleError(distance) + angleError);
                else
                    angle += Math.toRadians(angleError(distance) + angleError);
            }
            if (angle < 0)
                angle += 2 * Math.PI;
        }

        if (betterGamepad.right_bumper.pressed && !shoot && canShoot(follower.getPose()) != ShootingZone.NONE)
            shoot = true;
        else if (betterGamepad.right_bumper.pressed) {
            follower.startTeleopDrive(true);
            shoot = false;
        }

        if (shoot) {
            follower.holdPoint(new Pose(lastX, lastY, angle));
            if (follower.getPose().getX() >= lastX - errorX && follower.getPose().getX() <= lastX + errorX &&
                    follower.getPose().getY() >= lastY - errorY && follower.getPose().getY() <= lastY + errorY &&
                    follower.getPose().getHeading() >= angle - Math.toRadians(errorH)
                    && follower.getPose().getHeading() <= angle + Math.toRadians(errorH) && !tuning) {
                if (r) {
                    delay.resetTimer();
                    r = false;
                }
                if (delay.getElapsedTime() >= delayT)
                    shoot(shootMult(distance) + shootError);
                else
                    block.resetTimer();
            } else {
                for (DcMotorEx shooter : robot.shooters)
                    shooter.setVelocity(0);
                for (Servo blocker : robot.blockers)
                    blocker.setPosition(blockerBlockedPos);
                if (tuning)
                    robot.collector.setVelocity(0);
                else
                    robot.collector.setVelocity(vMax);
                block.resetTimer();
                r = true;
            }
        } else {
            if (limelightAim) {
                double headingError = angle - follower.getPose().getHeading();

                while (headingError > Math.PI)
                    headingError -= 2 * Math.PI;
                while (headingError < -Math.PI)
                    headingError += 2 * Math.PI;

                double turnPower = MathFunctions.clamp(headingError * 0.8, -0.45, 0.45);

                if (goalX == 12) {
                    follower.setTeleOpDrive(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            turnPower,
                            false);
                } else {
                    follower.setTeleOpDrive(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            turnPower,
                            false);
                }
            } else {
                drive(gamepad1);
            }

            for (DcMotorEx shooter : robot.shooters)
                shooter.setVelocity(0);
            for (Servo blocker : robot.blockers)
                blocker.setPosition(blockerBlockedPos);
            if (betterGamepad.left_trigger.held)
                robot.collector.setVelocity(-gamepad1.left_trigger * vMax);
            else if (betterGamepad.left_bumper.held)
                robot.collector.setVelocity(vMax);
            else
                robot.collector.setVelocity(0);
            lastX = follower.getPose().getX();
            lastY = follower.getPose().getY();
            block.resetTimer();
        }

        if (betterGamepad.triangle.pressed)
            if (goalX == 12)
                follower.setPose(start);
            else
                follower.setPose(start.mirror());

        follower.update();// dap

        telemetry.addData("robot X", follower.getPose().getX());
        telemetry.addData("robot Y", follower.getPose().getY());
        telemetry.addData("robot H", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("angle", Math.toDegrees(angle));
        telemetry.addData("distance", distance + " IN");
        telemetry.addData("shoot multiplier", shootMult(distance) + shootError);
        telemetry.addData("zone", canShoot(follower.getPose()));
        telemetry.addData("collector velocity", robot.collector.getVelocity());
        telemetry.addData("tuning", tuning);
        telemetry.addData("limelight aim", limelightAim);
        com.qualcomm.hardware.limelightvision.LLResult latestLimelightResult = robot.limelight.getLatestResult();
        boolean anyTagSeen = latestLimelightResult != null && latestLimelightResult.isValid()
                && !latestLimelightResult.getFiducialResults().isEmpty();

        telemetry.addData("Target Tag Seen", fiducial != null);
        telemetry.addData("Any Tag Seen", anyTagSeen);
        telemetry.addData("block", block.getElapsedTime());
        telemetry.addData("delay", delay.getElapsedTime());
        telemetry.addData("blockT", blockT);
        telemetry.addData("angle error close left RED", angleErrorRCL);
        telemetry.addData("angle error close left BLUE", angleErrorBCL);
        telemetry.addData("angle error close right RED", angleErrorRCR);
        telemetry.addData("angle error close right BLUE", angleErrorBCR);
        telemetry.addData("angle error far left RED", angleErrorRFL);
        telemetry.addData("angle error far left BLUE", angleErrorBFL);
        telemetry.addData("angle error far right RED", angleErrorRFR);
        telemetry.addData("angle error far right BLUE", angleErrorBFR);
        telemetry.addData("shoot error close", shootErrorC);
        telemetry.addData("shoot error far", shootErrorF);
        telemetry.addData("goalX", goalX);
    }

    public void drive(Gamepad gamepad) {
        if (goalX == 12)
            follower.setTeleOpDrive(
                    gamepad.left_stick_y,
                    gamepad.left_stick_x,
                    -gamepad.right_stick_x,
                    false);
        else
            follower.setTeleOpDrive(
                    -gamepad.left_stick_y,
                    -gamepad.left_stick_x,
                    -gamepad.right_stick_x,
                    false);
    }

    public void shoot(double multiplier) {
        for (DcMotorEx shooter : robot.shooters) {
            shooter.setVelocity(MathFunctions.clamp(multiplier, .01, 1) * vMax);
        }

        if (block.getElapsedTime() < preShootReverseT) {
            for (Servo blocker : robot.blockers)
                blocker.setPosition(blockerBlockedPos);
            robot.collector.setVelocity(collectorReverseMult * vMax);
        } else {

            for (Servo blocker : robot.blockers)
                blocker.setPosition(blockerOpenPos);

            if (block.getElapsedTime() >= blockT + preShootReverseT)
                robot.collector.setVelocity(vMax);
            else
                robot.collector.setVelocity(collectorReverseMult * vMax);
        }
    }

    public double angleError(double distance) {
        return MathFunctions.clamp(4.83415 * 1e-7 * Math.pow(distance, 4) - 0.000184899 * Math.pow(distance, 3)
                + 0.0251689 * Math.pow(distance, 2) - 1.4605 * distance + 40.69425, 6, 12);
    }

    public double shootMult(double distance) {
        double rawPower = MathFunctions
                .clamp(-3.28801 * 1e-7 * Math.pow(distance, 3) + 0.000115734 * Math.pow(distance, 2)
                        - 0.0123207 * distance + 1.13939, .725, .79);

        if (canShoot(follower.getPose()) == ShootingZone.CLOSE) {
            double currentVoltage = robot.batteryVoltageSensor.getVoltage();
            if (currentVoltage > 7.0) {
                double compensatedPower = rawPower * (referenceVoltage / currentVoltage);
                return MathFunctions.clamp(compensatedPower, 0.65, 0.85);
            }
        }
        return rawPower;
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
                { -halfW, -halfL }, { halfW, -halfL },
                { halfW, halfL }, { -halfW, halfL }
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
            if (isClose(corner))
                anyClose = true;
            if (isFar(corner))
                anyFar = true;
        }

        if (anyClose)
            return ShootingZone.CLOSE;
        if (anyFar)
            return ShootingZone.FAR;

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
