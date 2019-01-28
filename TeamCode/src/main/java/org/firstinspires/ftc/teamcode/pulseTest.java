package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Pulsing",group = "12596")
public class pulseTest extends OpMode {
    public CRServo srvCollectorL;
    public CRServo srvCollectorR;

    @Override
    public void init() {
        srvCollectorL = hardwareMap.crservo.get("srvCollectorL");
        srvCollectorR = hardwareMap.crservo.get("srvCollectorR");
        telemetry.addData("Hello", "Driver");
        telemetry.update();

    }

    @Override
    public void loop() {
        srvCollectorR.setPower(.7);
        srvCollectorL.setPower(.7);
    }
}