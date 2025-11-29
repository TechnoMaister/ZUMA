package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.backup;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorPowerB;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorPowerD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.collectorPowerN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.k;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rightTOL;
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
import org.firstinspires.ftc.teamcode.util.ThankYouVV.BetterGamepad;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends OpMode {
    public Hardware robot;
    public Follower follower;
    public Pose startingPose;
    public BetterGamepad betterGamepad;
    public Timer rumbling, rumbling2, block;
    public boolean direction, team;
    public AprilTagDetection id;
    public BasketLauncher launcher;
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

        if (id != null) {
            shooterSpeed = launcher.power(id.ftcPose.y-1.4, hardwareMap);
            gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            telemetry.addLine("I see!");
        }

        telemetry.addData("shooterPower", shooterSpeed);
        telemetry.addData("shooterVel", robot.leftShoot.getVelocity());
        telemetry.addData("backup", backup);
        if(id!=null)telemetry.addData("pos",id.center.x);

        drive(gamepad1);

        if (betterGamepad.right_trigger.pressed) direction = !direction;
        if (betterGamepad.cross.pressed) team = !team;

        if(betterGamepad.dpad_up.pressed && backup < 1) backup += .05;
        else if(betterGamepad.dpad_down.pressed && backup > 0) backup -= .05;

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

        if (betterGamepad.left_bumper.held) robot.collector.setPower(collectorPowerD);
        else if (!betterGamepad.right_bumper.held) robot.collector.setPower(0);

        if (betterGamepad.right_bumper.held && id != null) {
            if (id.center.x > tolerance) {
                robot.collector.setPower(0);
                robot.leftFront.setPower(k);
                robot.leftRear.setPower(k);
                robot.rightFront.setPower(-k);
                robot.rightRear.setPower(-k);
            } else if (id.center.x < tolerance-rightTOL) {
                robot.collector.setPower(0);
                robot.leftFront.setPower(-k);
                robot.leftRear.setPower(-k);
                robot.rightFront.setPower(k);
                robot.rightRear.setPower(k);
            } else {
                robot.leftFront.setPower(0);
                robot.leftRear.setPower(0);
                robot.rightFront.setPower(0);
                robot.rightRear.setPower(0);
                //shoot(backup);
            }
        }
        else if(betterGamepad.circle.held) shoot(backup);
        else {
            for (DcMotorEx shooterMotor : robot.shooters) shooterMotor.setPower(0);
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
        for (DcMotorEx shooterMotor : robot.shooters) shooterMotor.setPower(speed);
        if(block.getElapsedTime() >= blockT) {
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerOpenPos);
            robot.collector.setPower(collectorPowerN);
        }
        else {
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
            robot.collector.setPower(collectorPowerB);
        }


    }
}