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
    float hsvLValues[] = {0F, 0F, 0F};
    float hsvRValues[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;



    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false); // this is false here?
        telemetry.addData("Hello", "Driver");
        telemetry.addData("Yeet","yeet");
        telemetry.update();
        robot.liftAndHook.srvShift.setPosition(.94);
        //.84 is neutral
    }

    @Override
    public void loop() {
        double leftPower = (gamepad1.left_stick_y);
        double rightPower = (gamepad1.right_stick_y);
        double lift = 0;
        double flop = gamepad2.left_stick_y;

        Color.RGBToHSV((int) (robot.collector.sensorColorL.red() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorL.green() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorL.blue() * SCALE_FACTOR),
                hsvLValues);
        Color.RGBToHSV((int) (robot.collector.sensorColorR.red() * SCALE_FACTOR),
                    (int) (robot.collector.sensorColorR.green() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorR.blue() * SCALE_FACTOR),
                hsvRValues);

        // sets drive train half power
        if (gamepad1.right_trigger > 0.1) {
            leftPower = leftPower / 2;
            rightPower = rightPower / 2;
        }
        if (gamepad1.left_trigger > 0.1) {
            leftPower = leftPower * .75;
            rightPower = rightPower * .75;
        }

        // Sets deposits straight up
        if (gamepad2.y) {
            // move to 0 degrees.
            robot.collector.servoDepositL.setPosition(0);
            robot.collector.servoDepositR.setPosition(1);
        }

        if (gamepad2.a) {
            if (hsvRValues[0] > 100) {
                robot.collector.servoDepositL.setPosition(.62); // Deposit silver
            } else {
                robot.collector.servoDepositL.setPosition(.4); // Deposit Gold
            }
        }
        // Right side deposit
        if (gamepad2.a) {
            if (hsvLValues[0] > 100) { // Deposit silver mineral
                robot.collector.servoDepositR.setPosition(.35);
            } else {
                robot.collector.servoDepositR.setPosition(.57); // Deposit Gold mineral
            }
        }
        if (gamepad2.b) {
            if (hsvLValues[0] > 100) {
                robot.collector.servoDepositL.setPosition(.62); // Deposit silver
            } else {
                robot.collector.servoDepositL.setPosition(.4); // Deposit Gold
            }
        }
        // Right side deposit
        if (gamepad2.b) {
            if (hsvRValues[0] > 100) { // Deposit silver mineral
                robot.collector.servoDepositR.setPosition(.35);
            } else {
                robot.collector.servoDepositR.setPosition(.57); // Deposit Gold mineral
            }
        }

        // Lift with right stick up and down
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            lift = gamepad2.right_stick_y;
            robot.liftAndHook.mtrLift1.setPower(lift);
            robot.liftAndHook.mtrLift2.setPower(lift);
            robot.liftAndHook.mtrLift3.setPower(lift);
        } else {
            robot.liftAndHook.mtrLift1.setPower(0);
            robot.liftAndHook.mtrLift2.setPower(0);
            robot.liftAndHook.mtrLift3.setPower(0);
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
        // ball shifter

        robot.liftAndHook.srvShift.setPosition(.96);

        // Intake tubing
        if (gamepad1.right_trigger > 0.1) {
            leftPower = leftPower * .5;
            rightPower = rightPower * .5;
        }
        if (gamepad1.left_trigger > 0.1) {
            leftPower = leftPower * .75;
            rightPower = rightPower * .75;
        }


        // flickmode
        robot.driveTrain.mtrBL.setPower(leftPower);
        robot.driveTrain.mtrFL.setPower(leftPower);
        robot.driveTrain.mtrBR.setPower(rightPower);
        robot.driveTrain.mtrFR.setPower(rightPower);
        //robot.driveTrain.Tank(leftPower, rightPower);
        robot.liftAndHook.mtrFlop.setPower(flop);


        //Telemetry
        telemetry.addData("rightDistance (cm)",
                String.format(Locale.US, "%.02f", robot.collector.sensorDistanceR.getDistance(DistanceUnit.CM)));
        telemetry.addData("leftDistance (cm)",
                String.format(Locale.US, "%.02f", robot.collector.sensorDistanceL.getDistance(DistanceUnit.CM)));
        telemetry.addData("rightBlue ", robot.collector.sensorColorR.blue());
        telemetry.addData("leftBlue ", robot.collector.sensorColorL.blue());
        telemetry.addData("potentiometer data", robot.liftAndHook.angleTurned(robot.liftAndHook.potentiometer));
        telemetry.update();
    }
}


