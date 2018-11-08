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
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // sets drive train half power
        if (gamepad1.right_trigger > 0.1) {
            leftPower = leftPower / 2;
            rightPower = rightPower / 2;
        }

        // Sets deposits straight up
        if (gamepad2.y) {
            // move to 0 degrees.
            robot.liftAndHook.servoDepositL.setPosition(0);
            robot.liftAndHook.servoDepositR.setPosition(.88);
        }

        // Dead zone for drive train
        if (Math.abs(gamepad1.left_stick_y) > .1) {
            leftPower = leftPower;
        }
        if (Math.abs(gamepad1.right_stick_y) > .1) {
            rightPower = rightPower;
        }


        // Left side deposit
        if (robot.liftAndHook.sensorDistanceL.getDistance(DistanceUnit.CM) < 12.0) {
            if (gamepad2.a) {
                if (robot.liftAndHook.sensorColorL.blue() > 70) {
                    robot.liftAndHook.servoDepositL.setPosition(.66); // Deposit silver
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
                    robot.liftAndHook.servoDepositR.setPosition(.9);
                }
                else {
                    robot.liftAndHook.servoDepositR.setPosition(.33); // Deposit Gold mineral
                }
            }
        }

        if(gamepad2.a) {
            robot.liftAndHook.servoDepositR.setPosition(.9);
            robot.liftAndHook.servoDepositL.setPosition(.66);
        }
        if (gamepad2.b) {
            robot.liftAndHook.servoDepositR.setPosition(.33);
            robot.liftAndHook.servoDepositL.setPosition(.40);
        }
        if (gamepad2.y) {
            robot.liftAndHook.servoDepositR.setPosition(.88);
            robot.liftAndHook.servoDepositL.setPosition(0);
        }

        // Background change to help drivers know what's in the box
        if ((robot.liftAndHook.sensorDistanceR.getDistance(DistanceUnit.CM) < 12.0) && (robot.liftAndHook.sensorDistanceL.getDistance(DistanceUnit.CM) < 12.0)) {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.green(250));
                }
            });
        } else if ((robot.liftAndHook.sensorDistanceR.getDistance(DistanceUnit.CM) < 12.0)){
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.red(250));
                }
            });
        } else if ((robot.liftAndHook.sensorDistanceL.getDistance(DistanceUnit.CM) < 12.0)) {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.blue(250));
                }
            });
        }
        else {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.rgb(150, 150, 150));
                }
            });
        }

        // Lift with right trigger up, left trigger down
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            Lift = -gamepad2.right_stick_y;
        }
        else {
            robot.liftAndHook.mtrLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.liftAndHook.mtrLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Right trigger to rotate intake, left trigger spits out
        if (gamepad2.right_trigger > 0.1) {
            robot.collector.mtrIntake.setPower(-gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > 0.1) {
            robot.collector.mtrIntake.setPower(gamepad2.left_trigger);
        }
        else
        {
            robot.collector.mtrIntake.setPower(0);
        }

        // Set corresponding power to motors
        robot.liftAndHook.mtrLiftR.setPower(Lift);
        robot.liftAndHook.mtrLiftL.setPower(Lift);
        robot.driveTrain.Tank(rightPower, leftPower); // Tank Drive


        //Telemetry
        telemetry.addData("rightDistance (cm)",
                String.format(Locale.US, "%.02f", robot.liftAndHook.sensorDistanceR.getDistance(DistanceUnit.CM)));
        telemetry.addData("leftDistance (cm)",
                String.format(Locale.US, "%.02f", robot.liftAndHook.sensorDistanceL.getDistance(DistanceUnit.CM)));
        telemetry.addData("rightBlue ", robot.liftAndHook.sensorColorR.blue());
        telemetry.addData("leftBlue ", robot.liftAndHook.sensorColorL.blue());
        telemetry.addData("left stick", gamepad1.left_stick_y);
        telemetry.addData("right stick", gamepad1.right_stick_y);
        telemetry.update();
    }
}

