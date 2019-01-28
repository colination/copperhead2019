package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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

    public Servo servoDepositR;
    public Servo servoDepositL;
    public Servo srvMarker;

    public DistanceSensor sensorDistanceR;
    public DistanceSensor sensorDistanceL;
    public ColorSensor sensorColorR;
    public ColorSensor sensorColorL;

    public ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap ahwmap, Telemetry myTelemetry){
        super.init(ahwmap, myTelemetry);

        srvCollectorL = ahwmap.crservo.get("srvCollectorL");
        srvCollectorR = ahwmap.crservo.get("srvCollectorR");

        srvFlopL = ahwmap.crservo.get("srvFlopL");
        srvFlopL.setDirection(CRServo.Direction.REVERSE);
        srvFlopR = ahwmap.crservo.get("srvFlopR");

        servoDepositR = ahwmap.servo.get("servoDepositR");
        servoDepositL = ahwmap.servo.get("servoDepositL");
        srvMarker = ahwmap.servo.get("srvMarker");

        sensorColorR = ahwmap.colorSensor.get("sensorColorR");
        sensorColorL = ahwmap.colorSensor.get("sensorColorL");

        sensorDistanceR = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorColorR");
        sensorDistanceL = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorColorL");

    }
    public void markerDrop (double seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds/2) {
            srvFlopL.setPower(-.1);
            srvFlopR.setPower(-.1);
        }
        runtime.reset();
        while (runtime.seconds() < seconds) {
            srvCollectorR.setPower(-.7);
            srvCollectorL.setPower(-.7);
        }
    }
/*
    public void Extend(double Power){
        mtrExtendR.setPower(Power);
        mtrExtendL.setPower(Power);
    }

    public void Retract(double Power){
        mtrExtendR.setPower(-Power);
        mtrExtendL.setPower(-Power);
    }

    public void angle(double Power){
        srvFlopL.setPower(1);
        srvFlopR.setPower(1);
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
        privateTelemetry.addData("Depositing","Now");
        privateTelemetry.update();
        runtime.reset();
        while(runtime.seconds() < 1){
            angle(1);
        }
        runtime.reset();
        stop();

        while(runtime.seconds() < 1){
            collect(-1);
        }
        stop();

        runtime.reset();
        while(runtime.seconds() < 1){
            angle(-1);
        }
        stop();
    }
    public void park() {
        runtime.reset();
        while (runtime.seconds() < 1.0) {
            mtrExtendR.setPower(.5);
            mtrExtendL.setPower(.5);
        }
        mtrExtendR.setPower(0);
        mtrExtendL.setPower(0);
        while (runtime.seconds() < 4.0) {
            srvFlopR.setPower(.5);
            srvFlopR.setPower(.5);
        }
        srvFlopR.setPower(0);
        srvFlopR.setPower(0);
            while (runtime.seconds() < 7.0) {
                mtrExtendR.setPower(.5);
                mtrExtendL.setPower(.5);
            }
    }*/
}
