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
        robot.init(hardwareMap,telemetry);
        telemetry.addData("Hello","Driver");
        telemetry.update();
        robot.liftAndHook.reset();
        robot.liftAndHook.mtrLiftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftAndHook.mtrLiftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double leftPower  = (-gamepad1.left_stick_y);
        double rightPower = (-gamepad1.right_stick_y);
        double Lift = 0;
        double flop = gamepad2.left_stick_y;

        // sets drive train half power
        if (gamepad1.right_trigger > 0.1) {
            leftPower = leftPower / 2;
            rightPower = rightPower / 2;
        }

        /* Sets deposits straight up
        if (gamepad2.y) {
            // move to 0 degrees.
            robot.liftAndHook.servoDepositL.setPosition(.5);
            robot.liftAndHook.servoDepositR.setPosition(.88);
        }

        // Dead zone for drive train
        if (Math.abs(gamepad1.left_stick_y) > 0) {
            leftPower = leftPower;
        }
        if (Math.abs(gamepad1.right_stick_y) > 0) {
            rightPower = rightPower;
        }


         Left side deposit
        if (robot.collector.sensorDistanceL.getDistance(DistanceUnit.CM) < 12.0) {
            if (gamepad2.a) {
                if (robot.liftAndHook.sensorColorL.blue() > 70) {
                    robot.liftAndHook.servoDepositL.setPosition(.63); // Deposit silver
                }
                else {
                    robot.liftAndHook.servoDepositL.setPosition(.40); // Deposit Gold
                }
            }
        }

        // Right side deposit
        if (robot.liftAndHook.sensorDistanceR.getDistance(DistanceUnit.CM) < 12.0)  {
            if (gamepad2.a) {
                if (robot.liftAndHook.sensorColorR.blue() > 70) { // Deposit silver mineral
                    robot.liftAndHook.servoDepositR.setPosition(0);
                }
                else {
                    robot.liftAndHook.servoDepositR.setPosition(.33); // Deposit Gold mineral
                }
            }
        }*/

        // Lift with right trigger up, left trigger down
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            Lift = gamepad2.right_stick_y;
        }
        else {
            robot.liftAndHook.mtrLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.liftAndHook.mtrLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Right trigger to rotate intake, left trigger spits out
//        if (gamepad2.right_trigger > 0.1) {
//            robot.collector.mtrIntake.setPower(-gamepad2.right_trigger);
//        }
//        else if (gamepad2.left_trigger > 0.1) {
//            robot.collector.mtrIntake.setPower(gamepad2.left_trigger);
//        }
//        else
//        {
//            robot.collector.mtrIntake.setPower(0);
//        }
//        if (Math.abs(gamepad2.left_stick_y) > 0){
//            robot.collector.srvFlopL.setPower(-gamepad2.left_stick_y);
//        }
        if (gamepad1.left_trigger > 0.1) {
            leftPower = leftPower/2;
            rightPower = rightPower/2;
        }


        // Set corresponding power to motors

        robot.driveTrain.Tank(rightPower, leftPower); // Tank Drive
        //robot.collector.srvFlopL.setPower(flop); // Collector flop out
        robot.liftAndHook.mtrLiftR.setPower(Lift);
        robot.liftAndHook.mtrLiftL.setPower(Lift);


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

