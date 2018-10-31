package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class    Collector extends robotPart {
    public CRServo srvCollectorL;
    public CRServo srvCollectorR;
    public CRServo srvFlopL;
    public CRServo srvFlopR;
    public Servo brushSystem;

    public DcMotor mtrExtendL;
    public DcMotor mtrExtendR;
    public ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap ahwmap, Telemetry myTelemetry){
        super.init(ahwmap, myTelemetry);

        srvCollectorL = ahwmap.crservo.get("srvCollectorL");
        srvCollectorR = ahwmap.crservo.get("srvCollectorR");
        srvFlopL = ahwmap.crservo.get("srvFlopL");
        srvFlopR = ahwmap.crservo.get("srvFlopR");
        brushSystem = ahwmap.servo.get("brush");

        mtrExtendL = ahwmap.dcMotor.get("mtrExtenderL");
        mtrExtendR = ahwmap.dcMotor.get("mtrExtenderR");
        mtrExtendR.setDirection(DcMotorSimple.Direction.REVERSE);
        srvCollectorR.setDirection(CRServo.Direction.REVERSE);
        srvFlopR.setDirection(CRServo.Direction.REVERSE);
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
        srvCollectorL.setPower(-power);
        srvCollectorR.setPower(power);
    }
    public void stop(){
        mtrExtendL.setPower(0);
        mtrExtendR.setPower(0);
        srvCollectorL.setPower(0);
        srvCollectorR.setPower(0);
        srvFlopL.setPower(0);
        srvFlopL.setPower(0);
    }

    public void depositMarker(){
        runtime.reset();
        while(runtime.seconds() < 1){

        }
        runtime.reset();
        stop();

        while(runtime.seconds() < 1){
            collect(-1);
        }
        stop();

        runtime.reset();
        while(runtime.seconds() < 1){
            
        }
        stop();
    }
}
