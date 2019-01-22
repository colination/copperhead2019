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
    int isSlow = 1;

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
        double Lift = gamepad2.right_stick_y;
        double flop = gamepad2.left_stick_y;
        //int slow;

        // sets drive train half power
        if (gamepad1.right_trigger > 0.1) {
            leftPower = leftPower * .35;
            rightPower = rightPower * .35;
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
            robot.collector.servoDepositL.setPosition(0);
            robot.collector.servoDepositR.setPosition(1);
        }

        // Balls - Left side deposit
        if (gamepad2.a) {
            if (leftHue() > 85) {
                robot.collector.servoDepositL.setPosition(.4); // Deposit silver
            } else {
                robot.collector.servoDepositL.setPosition(.21); // Deposit Gold
            }
        }
        // Balls - Right side deposit
        if (gamepad2.a) {
            if (righttHue() > 90) { // Deposit silver mineral
                robot.collector.servoDepositR.setPosition(.6); // Deposit Silver
            } else {
                robot.collector.servoDepositR.setPosition(.79); // Deposit Gold mineral
            }
        }
        // Cubes - Left side deposit
        if (gamepad2.b) {
            if (leftHue() < 85) {
                robot.collector.servoDepositL.setPosition(.4); // Deposit silver
            } else {
                robot.collector.servoDepositL.setPosition(.21); // Deposit Gold
            }
        }
        // Cubes - right side deposit
        if (gamepad2.b) {
            if (righttHue() < 90) { // Deposit silver mineral
                robot.collector.servoDepositR.setPosition(.6);
            } else {
                robot.collector.servoDepositR.setPosition(.79); // Deposit Gold mineral
            }
        }

        if (Math.abs(flop) > .1) {
            flop = flop;
        } else {
            robot.liftAndHook.mtrFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Trigger intake
        if (Math.abs(gamepad2.right_trigger) > .05) {
            robot.collector.srvCollectorR.setPower(gamepad2.right_trigger * .7);
            robot.collector.srvCollectorL.setPower(gamepad2.right_trigger * .7);
        } else if (Math.abs(gamepad2.left_trigger) > .05) {
            robot.collector.srvCollectorR.setPower(gamepad2.left_trigger * -.7);
            robot.collector.srvCollectorL.setPower(gamepad2.left_trigger * -.7);
        } else {
            robot.collector.srvCollectorR.setPower(0);
            robot.collector.srvCollectorL.setPower(0);
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

        //Lift
        if (gamepad1.a) {
            Lift = -.25;
        } else if (gamepad1.x) {
            Lift = .25;
        } else if (Math.abs(gamepad2.right_stick_y) > .01) {
            Lift = Lift;
        } else {
            Lift = 0;
        }

        /* Lift with right stick up and down
        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
            Lift = gamepad2.right_stick_y;
            if (gamepad2.dpad_up) {
                isSlow *= -1;
                telemetry.addData("isSlow:", isSlow);
                telemetry.update();
            }
            if ((isSlow < 0) && (Lift > 0)) {
                Lift = .25;
                telemetry.addLine().addData("isSlow:", isSlow);
                robot.liftAndHook.mtrLift1.setPower(Lift);
                robot.liftAndHook.mtrLift2.setPower(Lift);
                robot.liftAndHook.mtrLift3.setPower(Lift);
            }
            else {
                robot.liftAndHook.mtrLift1.setPower(Lift);
                robot.liftAndHook.mtrLift2.setPower(Lift);
                robot.liftAndHook.mtrLift3.setPower(Lift);
            }
        } else {
            robot.liftAndHook.mtrLift1.setPower(0);
            robot.liftAndHook.mtrLift2.setPower(0);
            robot.liftAndHook.mtrLift3.setPower(0);
            robot.liftAndHook.mtrLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.liftAndHook.mtrLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.liftAndHook.mtrLift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }*/
//        if (Math.abs(gamepad2.right_stick_y) > 0.1) {
//            Lift = Lift;
//        }
//        else {
//            Lift = 0;
//        }
        robot.liftAndHook.mtrLift1.setPower(Lift);
        robot.liftAndHook.mtrLift2.setPower(Lift);
        robot.liftAndHook.mtrLift3.setPower(Lift);
        robot.liftAndHook.mtrLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftAndHook.mtrLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftAndHook.mtrLift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // marker
        if (gamepad1.y) {
            robot.collector.srvMarker.setPosition(.4);
        }
        // Motor rotate
        robot.liftAndHook.mtrFlop.setPower(flop);


        /*Telemetry
        telemetry.addData("rightDistance (cm)",
                String.format(Locale.US, "%.02f", robot.collector.sensorDistanceR.getDistance(DistanceUnit.CM)));
        telemetry.addData("leftDistance (cm)",
                String.format(Locale.US, "%.02f", robot.collector.sensorDistanceL.getDistance(DistanceUnit.CM)));
        telemetry.addData("rightBlue ", hsvRValues[0]);
        telemetry.addData("leftBlue ", hsvLValues[0]);*/
        //telemetry.addData("mtrFlopEncoders",robot.liftAndHook.mtrFlop.getCurrentPosition());
        telemetry.update();
    }


    public double righttHue() {
        // Hue conversion
        Color.RGBToHSV((int) (robot.collector.sensorColorL.red() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorL.green() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorL.blue() * SCALE_FACTOR),
                hsvLValues);
        return hsvLValues[0];
    }

    public double leftHue() {
        // Hue conversion
        Color.RGBToHSV((int) (robot.collector.sensorColorR.red() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorR.green() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorR.blue() * SCALE_FACTOR),
                hsvRValues);
        return hsvRValues[0];
    }
}


