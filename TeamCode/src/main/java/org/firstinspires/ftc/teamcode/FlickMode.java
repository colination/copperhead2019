package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "Flicko Mode",group = "12596")
public class FlickMode extends OpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    float hsvLValues[] = {0F, 0F, 0F};
    float hsvRValues[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;

    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DcMotor mtrBL = null;
    public DcMotor mtrBR = null;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false); // this is false here?
        telemetry.addData("Hello", "Driver");
        telemetry.addData("Yeet","yeet");
        telemetry.update();
        // Motors
        mtrFL = hardwareMap.dcMotor.get("mtrFL");
        mtrFR = hardwareMap.dcMotor.get("mtrFR");
        mtrBL = hardwareMap.dcMotor.get("mtrBL");
        mtrBR = hardwareMap.dcMotor.get("mtrBR");
        // Set motor directions
        mtrFL.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrBL.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrFR.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrBR.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set motors to not use encoders until specified
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        double leftPower = gamepad1.left_stick_y;
        double rightPower = gamepad1.right_stick_y;
        double Lift = 0;
        double flop = gamepad2.left_stick_y;

        // Hue conversion
        Color.RGBToHSV((int) (robot.liftAndHook.sensorColorL.red() * SCALE_FACTOR),
                (int) (robot.liftAndHook.sensorColorL.green() * SCALE_FACTOR),
                (int) (robot.liftAndHook.sensorColorL.blue() * SCALE_FACTOR),
                hsvLValues);
        Color.RGBToHSV((int) (robot.liftAndHook.sensorColorR.red() * SCALE_FACTOR),
                (int) (robot.liftAndHook.sensorColorR.green() * SCALE_FACTOR),
                (int) (robot.liftAndHook.sensorColorR.blue() * SCALE_FACTOR),
                hsvRValues);

        // sets drive train half power
        if (gamepad1.right_trigger > 0.1) {
            leftPower = leftPower * .35;
            rightPower = rightPower * .5;
        }
        if (gamepad1.left_trigger > 0.1) {
            leftPower = leftPower * .75;
            rightPower = rightPower * .75;
        }

        // flickmode
        mtrBL.setPower(leftPower);
        mtrFL.setPower(leftPower);
        mtrBR.setPower(rightPower);
        mtrFR.setPower(rightPower);

        // Sets deposits straight up
        if (gamepad2.y) {
            // move to 0 degrees.
            robot.liftAndHook.servoDepositL.setPosition(0);
            robot.liftAndHook.servoDepositR.setPosition(1);
        }

        if (gamepad2.a) {
            if (hsvRValues[0] > 100) {
                robot.liftAndHook.servoDepositL.setPosition(.62); // Deposit silver
            } else {
                robot.liftAndHook.servoDepositL.setPosition(.4); // Deposit Gold
            }
        }
        // Right side deposit
        if (gamepad2.a) {
            if (hsvLValues[0] > 100) { // Deposit silver mineral
                robot.liftAndHook.servoDepositR.setPosition(.35);
            } else {
                robot.liftAndHook.servoDepositR.setPosition(.57); // Deposit Gold mineral
            }
        }
        if (gamepad2.b) {
            if (hsvRValues[0] < 100) {
                robot.liftAndHook.servoDepositL.setPosition(.62); // Deposit silver
            } else {
                robot.liftAndHook.servoDepositL.setPosition(.4); // Deposit Gold
            }
        }
        // Right side deposit
        if (gamepad2.b) {
            if (hsvLValues[0] < 100) { // Deposit silver mineral
                robot.liftAndHook.servoDepositR.setPosition(.35);
            } else {
                robot.liftAndHook.servoDepositR.setPosition(.57); // Deposit Gold mineral
            }
        }

        // Lift with right stick up and down
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            Lift = gamepad2.right_stick_y;
            robot.liftAndHook.mtrLift1.setPower(Lift);
            robot.liftAndHook.mtrLift2.setPower(Lift);
            robot.liftAndHook.mtrLift3.setPower(Lift);
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

        // Lift with right stick up and down
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            Lift = gamepad2.right_stick_y;
            robot.liftAndHook.mtrLift1.setPower(Lift);
            robot.liftAndHook.mtrLift2.setPower(Lift);
            robot.liftAndHook.mtrLift3.setPower(Lift);
        } else {
            robot.liftAndHook.mtrLift1.setPower(0);
            robot.liftAndHook.mtrLift2.setPower(0);
            robot.liftAndHook.mtrLift3.setPower(0);
            robot.liftAndHook.mtrLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.liftAndHook.mtrLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.liftAndHook.mtrLift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        // marker
        if (gamepad1.y) {
            robot.collector.srvMarker.setPosition(.4);
        }
        // Motor rotate
        robot.liftAndHook.mtrFlop.setPower(flop);


        //Telemetry
        telemetry.addData("rightDistance (cm)",
                String.format(Locale.US, "%.02f", robot.collector.sensorDistanceR.getDistance(DistanceUnit.CM)));
        telemetry.addData("leftDistance (cm)",
                String.format(Locale.US, "%.02f", robot.collector.sensorDistanceL.getDistance(DistanceUnit.CM)));
        telemetry.addData("rightBlue ", hsvRValues[0]);
        telemetry.addData("leftBlue ", hsvLValues[0]);
        telemetry.update();
    }
}


