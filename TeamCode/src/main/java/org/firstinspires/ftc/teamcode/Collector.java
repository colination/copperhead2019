package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class    Collector extends robotPart {
    public CRServo csrvCollectorL;
    public CRServo csrvCollectorR;
    public Servo srvFlopL;
    public Servo srvFlopR;

    public DcMotor mtrExtendL;
    public DcMotor mtrExtendR;

    public void init(HardwareMap ahwmap, Telemetry myTelemetry){
        super.init(ahwmap, myTelemetry);

        csrvCollectorL = ahwmap.crservo.get("csrvCollectorL");
        csrvCollectorR = ahwmap.crservo.get("csrvCollectorR");
        srvFlopL = ahwmap.servo.get("srvFlopL");
        srvFlopR = ahwmap.servo.get("srvFlopR");

        mtrExtendL = ahwmap.dcMotor.get("mtrExtenderL");
        mtrExtendR = ahwmap.dcMotor.get("mtrExtenderR");
        mtrExtendR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Extend(double Power){
        mtrExtendR.setPower(Power);
        mtrExtendL.setPower(Power);
    }

    public void Retract(double Power){
        mtrExtendR.setPower(-Power);
        mtrExtendL.setPower(-Power);
    }
    public void collect(double power){
        csrvCollectorL.setPower(-power);
        csrvCollectorR.setPower(power);
    }
}
