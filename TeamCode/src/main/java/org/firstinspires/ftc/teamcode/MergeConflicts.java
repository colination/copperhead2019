package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Flicko Mode",group = "12596")
public class MergeConflicts extends OpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DcMotor mtrBL = null;
    public DcMotor mtrBR = null;


    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false); // this is false here?
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
        //
        telemetry.addData("Hello", "Driver");
        telemetry.addData("Yeet", "yeet");
        telemetry.update();
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
    }
}