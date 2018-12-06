package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "TankTele",group = "12596")
public class TankBotTeleOp extends OpMode {
    CopperHeadRobot robot = new CopperHeadRobot();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false); // this is false here?
        telemetry.addData("Hello", "Driver");
        telemetry.addData("Yeet","yeet");
        telemetry.update();
    }

    @Override
    public void loop() {
        double leftPower = (gamepad1.left_stick_y);
        double rightPower = (gamepad1.right_stick_y);
        double Lift = 0;
        double flop = gamepad2.left_stick_y;

        // sets drive train half power
        if (gamepad1.right_trigger > 0.1) {
            leftPower = leftPower / 2;
            rightPower = rightPower / 2;
        }

        // Sets deposits straight up
        if (gamepad2.y) {
            // move to 0 degrees.
            robot.liftAndHook.servoDepositL.setPosition(.5);
            robot.liftAndHook.servoDepositR.setPosition(.88);
        }

        // Left side deposit
        if (robot.liftAndHook.sensorDistanceL.getDistance(DistanceUnit.CM) < 12.0) {
            if (gamepad2.a) {
                if (robot.liftAndHook.sensorColorL.blue() > 70) {
                    robot.liftAndHook.servoDepositL.setPosition(.63); // Deposit silver
                } else {
                    robot.liftAndHook.servoDepositL.setPosition(.40); // Deposit Gold
                }
            }
        }

        // Right side deposit
        if (robot.liftAndHook.sensorDistanceR.getDistance(DistanceUnit.CM) < 12.0) {
            if (gamepad2.a) {
                if (robot.liftAndHook.sensorColorR.blue() > 70) { // Deposit silver mineral
                    robot.liftAndHook.servoDepositR.setPosition(0);
                } else {
                    robot.liftAndHook.servoDepositR.setPosition(.33); // Deposit Gold mineral
                }
            }
        }

        // Lift with right stick up and down
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            Lift = gamepad2.right_stick_y;
        } else {
            robot.liftAndHook.mtrLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.liftAndHook.mtrLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.liftAndHook.mtrLift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (Math.abs(flop) > .1) {
            flop = flop;
        }
        else {
            robot.liftAndHook.mtrFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //bumper intake
        if (gamepad2.right_trigger > 0.1) {
            robot.collector.srvFlopR.setPower(.5);
            robot.collector.srvFlopL.setPower(.5);}
        else if (gamepad2.left_trigger > 0.1) {
            robot.collector.srvFlopR.setPower(-.5);
            robot.collector.srvFlopL.setPower(-.5);
        }
        else {
            robot.collector.srvFlopR.setPower(0);
            robot.collector.srvFlopL.setPower(0);
        }

        if (gamepad2.right_bumper) {
            robot.collector.srvCollectorR.setPower(.7);
            robot.collector.srvCollectorL.setPower(.7);
        }
        else if (gamepad2.left_bumper){
            robot.collector.srvCollectorR.setPower(-.7);
            robot.collector.srvCollectorL.setPower(-.7);
        }
        else {
            robot.collector.srvCollectorR.setPower(0);
            robot.collector.srvCollectorL.setPower(0);
        }

        // Intake tubing
//        if (gamepad2.right_bumper) {
//            robot.collector.srvCollectorR.setPower(.7);
//            robot.collector.srvCollectorL.setPower(.7);
//            telemetry.addData("Yeet",5);
//        }
//        // Spit out tubing
//        else if (gamepad2.left_bumper) {
//            robot.collector.srvCollectorR.setPower(-.7);
//            robot.collector.srvCollectorL.setPower(-.7);
//            telemetry.addData("yeet",-5);
//        }
//        else {
//            robot.collector.srvCollectorR.setPower(0);
//            robot.collector.srvCollectorL.setPower(0);
//        }

        // Set corresponding power to motors
        robot.driveTrain.Tank(leftPower, rightPower); // Tank Drive
        robot.liftAndHook.mtrLift1.setPower(Lift);
        robot.liftAndHook.mtrLift2.setPower(Lift);
        robot.liftAndHook.mtrLift3.setPower(Lift);
        robot.liftAndHook.mtrFlop.setPower(flop);


        //Telemetry
        telemetry.addData("rightDistance (cm)",
                String.format(Locale.US, "%.02f", robot.collector.sensorDistanceR.getDistance(DistanceUnit.CM)));
        telemetry.addData("leftDistance (cm)",
                String.format(Locale.US, "%.02f", robot.collector.sensorDistanceL.getDistance(DistanceUnit.CM)));
        telemetry.addData("rightBlue ", robot.collector.sensorColorR.blue());
        telemetry.addData("leftBlue ", robot.collector.sensorColorL.blue());
        telemetry.update();
    }
}


