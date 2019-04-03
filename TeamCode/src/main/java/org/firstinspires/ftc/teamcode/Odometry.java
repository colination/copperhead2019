package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "odomo",group = "12596")
public class Odometry extends OpMode {

    public DcMotor odom1 = null;

    @Override
    public void init() {
        // Motors
        odom1 = hardwareMap.dcMotor.get("odom1");
        // Set motors to not use encoders until specified
        odom1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("Odometry Ticks", odom1.getCurrentPosition());
        telemetry.update();
    }
}