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
        telemetry.addData("Yeet", "yeet");
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

        // sets drive train half power
        if (gamepad1.left_trigger > 0.1) {
            leftPower = leftPower * .75;
            rightPower = rightPower * .75;
        }
        if (gamepad1.right_trigger > 0.1) {
            leftPower = leftPower * .35;
            rightPower = rightPower * .35;
        }

        // flickmode
        mtrBL.setPower(leftPower);
        mtrFL.setPower(leftPower);
        mtrBR.setPower(rightPower);
        mtrFR.setPower(rightPower);

        // Sets deposits straight up
        if (gamepad2.y) {
            // move to 0 degrees.
            robot.collector.servoDepositL.setPosition(0);
            robot.collector.servoDepositR.setPosition(1);
        }
        // Ball Side left side deposit
        if (gamepad2.a) {
            if (leftHue() > 90) {
                robot.collector.servoDepositL.setPosition(.37); // Deposit silver
            } else {
                robot.collector.servoDepositL.setPosition(.21); // Deposit Gold
            }
        }
        // Ball Side Right side deposit
        if (gamepad2.a) {
            if (rightHue() > 90) {
                robot.collector.servoDepositR.setPosition(.53); // Deposit silver mineral
            } else {
                robot.collector.servoDepositR.setPosition(.72); // Deposit Gold mineral
            }
        }
        // Cube side left side deposit
        if (gamepad2.b && !gamepad2.start) {
            if (leftHue() < 90) {
                robot.collector.servoDepositL.setPosition(.37); // Deposit silver
            } else {
                robot.collector.servoDepositL.setPosition(.21); // Deposit Gold
            }
        }
        // Cube side Right side deposit
        if (gamepad2.b && !gamepad2.start) {
            if (rightHue() < 90) {
                robot.collector.servoDepositR.setPosition(.53); // Deposit silver mineral
            } else {
                robot.collector.servoDepositR.setPosition(.72); // Deposit Gold mineral
            }
        }

        if (gamepad2.x && gamepad2.dpad_right) {
            robot.liftAndHook.goInches(24,.6, 5);
            if (leftHue() < 90) {
                robot.collector.servoDepositL.setPosition(.37); // Deposit silver
            } else {
                robot.collector.servoDepositL.setPosition(.21); // Deposit Gold
            }
            if (rightHue() < 90) {
                robot.collector.servoDepositR.setPosition(.53); // Deposit silver mineral
            } else {
                robot.collector.servoDepositR.setPosition(.72); // Deposit Gold mineral
            }
            robot.liftAndHook.goInches(-24, .3, 5);
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
        }
        if (flop > .05) {
            flop = flop * .6;
        } else if (flop < -.05) {
            flop = flop * .85;
        } else {
            flop = 0;
        }

        if (gamepad2.dpad_left && gamepad2.b)
        {
            macroRelatch();
        }
        //bumper rotate
        if (gamepad2.right_bumper) {
            robot.collector.srvFlopR.setPower(1);
            robot.collector.srvFlopL.setPower(1);
        } else if (gamepad2.left_bumper) {
            robot.collector.srvFlopR.setPower(-1);
            robot.collector.srvFlopL.setPower(-1);
        } else {
            robot.collector.srvFlopR.setPower(0);
            robot.collector.srvFlopL.setPower(0);
        }
        // Trigger intake
        if (gamepad2.right_trigger > .05) {
            robot.collector.srvCollectorR.setPower(gamepad2.right_trigger * .7);
            robot.collector.srvCollectorL.setPower(gamepad2.right_trigger * .7);
        } else if (gamepad2.left_trigger > .05) {
            robot.collector.srvCollectorR.setPower(gamepad2.left_trigger * -.7);
            robot.collector.srvCollectorL.setPower(gamepad2.left_trigger * -.7);
        } else {
            robot.collector.srvCollectorR.setPower(0);
            robot.collector.srvCollectorL.setPower(0);
        }

        // Motor rotate
        robot.liftAndHook.mtrFlop.setPower(flop);

        // Marker servo
        if (gamepad1.y) {
            robot.collector.srvMarker.setPosition(0.3);
        }
        if (gamepad1.b) {
            robot.liftAndHook.csrvPin.setPower(1);
        }
        if (gamepad1.x) {
            robot.liftAndHook.csrvPin.setPower(0);
        }

        // Ball Shifter
        if (gamepad2.dpad_left) { //neutral
            robot.liftAndHook.srvShift.setPosition(.96);
        }
        if (gamepad2.dpad_down) { //neutral
            robot.liftAndHook.srvShift.setPosition(.84);
        }
        if (gamepad2.dpad_right) { //neutral
            robot.liftAndHook.srvShift.setPosition(.79);
        }

//        telemetry.addData("left deposit", robot.collector.sensorDistanceL.getDistance(DistanceUnit.CM));
//        telemetry.addData("right deposit", robot.collector.sensorDistanceR.getDistance(DistanceUnit.CM));
//        telemetry.update();
    }
    public double leftHue() {
        // Hue conversion
        Color.RGBToHSV((int) (robot.collector.sensorColorR.red() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorR.green() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorR.blue() * SCALE_FACTOR),
                hsvRValues);
        return hsvRValues[0];
    }
    public double rightHue() {
        // Hue conversion
        Color.RGBToHSV((int) (robot.collector.sensorColorL.red() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorL.green() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorL.blue() * SCALE_FACTOR),
                hsvLValues);
        return hsvLValues[0];
    }

    public void macroRelatch() {
        robot.liftAndHook.goInches(18, 1, 5);
        robot.liftAndHook.goInches(-18, 1, 5);
    }
}