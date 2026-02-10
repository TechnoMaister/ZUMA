package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.backup;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.ThankYouVV.BetterGamepad;

//@Disabled
@TeleOp(name = "Test Veric", group = "Util")
public class TestVeric extends OpMode {

    public Hardware robot;
    public BetterGamepad betterGamepad;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);
        betterGamepad = new BetterGamepad(gamepad1);
    }

    @Override
    public void loop() {
        betterGamepad.update();
        /*if(betterGamepad.dpad_up.pressed && backup < 1) backup += .05;
        else if(betterGamepad.dpad_down.pressed && backup > 0) backup -= .05;
        if (betterGamepad.left_bumper.held) {
            for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(999999999);
            robot.collector.setVelocity(999999999);
        } else {
            for (DcMotorEx shooter : robot.shooters) shooter.setVelocity(0);
            robot.collector.setVelocity(0);
        }
        if (betterGamepad.cross.held) {
            for (DcMotorEx shooter : robot.shooters) shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addLine("STOP_AND_RESET_ENCODER");
        }
        if(betterGamepad.circle.held) {
            for (DcMotorEx shooter : robot.shooters) shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addLine("RUN_USING_ENCODER");
        }
        telemetry.addData("left shoot vel", robot.leftShoot.getVelocity());
        telemetry.addData("right shoot vel", robot.rightShoot.getVelocity());
        telemetry.addData("collector velocity", robot.collector.getVelocity());*/
        /*if(betterGamepad.dpad_up.held) robot.leftFront.setPower(1);
        else robot.leftFront.setPower(0);
        if(betterGamepad.dpad_left.held) robot.rightFront.setPower(1);
        else robot.rightFront.setPower(0);
        if(betterGamepad.dpad_down.held) robot.leftRear.setPower(1);
        else robot.leftRear.setPower(0);
        if(betterGamepad.dpad_right.held) robot.rightRear.setPower(1);
        else robot.rightRear.setPower(0);
        telemetry.addData("left front(dpad up)", robot.leftFront.getVelocity());
        telemetry.addData("right front(dpad left)", robot.rightFront.getVelocity());
        telemetry.addData("left rear(dpad down)", robot.leftRear.getVelocity());
        telemetry.addData("right rear(dpad right)", robot.rightRear.getVelocity());*/
        if(betterGamepad.dpad_up.held) for (DcMotorEx chassisMotor : robot.chassis) {
            robot.leftFront.setPower(1);
            robot.rightFront.setPower(1);
        }
        else if(betterGamepad.dpad_down.held) {
            robot.leftRear.setPower(1);
            robot.rightRear.setPower(1);
        }
        else if(betterGamepad.dpad_left.held) {
            for (DcMotorEx chassisMotor : robot.chassis) chassisMotor.setPower(1);
            //for (DcMotorEx chassisMotor : robot.rightChassis) chassisMotor.setPower(0);
        }
        else if(betterGamepad.dpad_right.held) {
            for (DcMotorEx chassisMotor : robot.leftChassis) chassisMotor.setPower(0);
            for (DcMotorEx chassisMotor : robot.rightChassis) chassisMotor.setPower(1);
        } else if(betterGamepad.cross.held) {
            robot.leftFront.setPower(1);
            robot.rightRear.setPower(1);
        } else if(betterGamepad.circle.held) {
            robot.rightFront.setPower(1);
            robot.leftRear.setPower(1);
        } else for (DcMotorEx chassisMotor : robot.chassis) chassisMotor.setPower(0);
    }
}
