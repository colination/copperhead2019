package org.firstinspires.ftc.teamcode;

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
    }

    @Override
    public void loop() {
        double leftPower  = -gamepad1.left_stick_y ;
        double rightPower = -gamepad1.right_stick_y ;
        double Lift = -gamepad1.right_stick_y;
        double extend = - gamepad2.right_stick_y;
        double flop = gamepad2.left_stick_y;

        //color sorted teleop, use once color sensors are wired
        // Sets deposits straight up

        if (gamepad2.y) {
            // move to 0 degrees.
            robot.liftAndHook.servoDepositL.setPosition(0);
            robot.liftAndHook.servoDepositR.setPosition(.85);
        }

        // Left side deposit
        if (robot.liftAndHook.sensorDistanceL.getDistance(DistanceUnit.CM) < 12.0) {
            if (gamepad1.a) {
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
            if (gamepad1.a) {
                if (robot.liftAndHook.sensorColorR.blue() > 70) { // Deposit silver mineral
                    robot.liftAndHook.servoDepositR.setPosition(.15);
                }
                else {
                    robot.liftAndHook.servoDepositR.setPosition(.40); // Deposit Gold mineral
                }
            }
        }

        // Lift with right joystick and holding right trigger
        if (gamepad1.right_trigger > 0.1) {
            leftPower = 0;
            rightPower = 0;
            if (Math.abs(Lift) > 0.1) {
                robot.liftAndHook.mtrLiftR.setPower(Lift);
                robot.liftAndHook.mtrLiftL.setPower(Lift);
            }
        }
        else {
            robot.liftAndHook.mtrLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.liftAndHook.mtrLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Extending motors with right joystick
        if (Math.abs(extend) > 0.1) {
            extend = extend;
        }
        else {
            robot.collector.mtrExtendL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.collector.mtrExtendR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Flopping out collector with left Joystick
        if (Math.abs(flop) > 0.1) {
            flop = flop;
        }
        else {
            flop = 0;
        }

        // Right trigger to rotate intake, left trigger spits out
        if (gamepad2.right_trigger > 0.1) {
            robot.collector.srvCollectorR.setDirection(CRServo.Direction.FORWARD);
            robot.collector.srvCollectorL.setDirection(CRServo.Direction.REVERSE);
            robot.collector.srvCollectorL.setPower(1.0);
            robot.collector.srvCollectorR.setPower(1.0);
        }
        else if (gamepad2.left_trigger> 0.1) {
            robot.collector.srvCollectorR.setDirection(CRServo.Direction.REVERSE);
            robot.collector.srvCollectorL.setDirection(CRServo.Direction.FORWARD);
            robot.collector.srvCollectorL.setPower(1.0);
            robot.collector.srvCollectorR.setPower(1.0);
        }
        else
        {
            robot.collector.srvCollectorL.setPower(0);
            robot.collector.srvCollectorR.setPower(0);
        }

        //50% speed for depositing control
        if (gamepad1.left_trigger > 0.1) {
            leftPower  = -gamepad1.left_stick_y / 2 ;
            rightPower = -gamepad1.right_stick_y / 2 ;
        }

        // Set corresponding power to motors
        robot.driveTrain.Tank(rightPower, leftPower); // Tank Drive
        robot.collector.mtrExtendL.setPower(extend/2); // Collector extension
        robot.collector.mtrExtendR.setPower(extend/2);
        robot.collector.srvFlopL.setPower(flop); // Collector flop out
        robot.collector.srvFlopR.setPower(flop);
        /*
        if (gamepad1.y) {
            // move to 0 degrees.
            robot.liftAndHook.servoDepositL.setPosition(0);
            robot.liftAndHook.servoDepositR.setPosition(.85);
        }
        if (gamepad1.b) {
            // Deposit Gold
            robot.liftAndHook.servoDepositL.setPosition(.4);
            robot.liftAndHook.servoDepositR.setPosition(.4);
        }
        if ( gamepad1.x) {
            robot.liftAndHook.servoDepositL.setPosition(.66);
            robot.liftAndHook.servoDepositR.setPosition(.4);
        }
        if (gamepad1.a) {
            // Deposit silver
            robot.liftAndHook.servoDepositL.setPosition(.66);
            robot.liftAndHook.servoDepositR.setPosition(.15);
        }/*
        if (gamepad1.y) {
            robot.collector.srvCollectorL.setPosition(0);
            robot.collector.srvCollectorR.setPosition(0);
        }
        if (gamepad1.a) {
            robot.collector.srvCollectorR.setPosition(.55);
            robot.collector.srvCollectorL.setPosition(1);
        }*/


        //Telemetry
        telemetry.addData("rightDistance (cm)",
                String.format(Locale.US, "%.02f", robot.liftAndHook.sensorDistanceR.getDistance(DistanceUnit.CM)));
        telemetry.addData("leftDistance (cm)",
                String.format(Locale.US, "%.02f", robot.liftAndHook.sensorDistanceL.getDistance(DistanceUnit.CM)));
        telemetry.addData("rightBlue ", robot.liftAndHook.sensorColorR.blue());
        telemetry.addData("leftBlue ", robot.liftAndHook.sensorColorL.blue());
        telemetry.addData("liftLTicks", robot.liftAndHook.mtrLiftL.getCurrentPosition());
        telemetry.addData("liftRTicks", robot.liftAndHook.mtrLiftR.getCurrentPosition());
        telemetry.update();

    }
}

