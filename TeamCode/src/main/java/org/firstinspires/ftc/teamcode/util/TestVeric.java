package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.backup;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.ThankYouVV.BetterGamepad;

@Disabled
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
        if(betterGamepad.dpad_up.pressed && backup < 1) backup += .05;
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
        telemetry.addData("collector velocity", robot.collector.getVelocity());
    }
}
