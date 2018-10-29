package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftAndHook extends robotPart {
    //Motors
    public DcMotor mtrLiftR;
    public DcMotor mtrLiftL;

    //Servos
    public Servo servoDepositR;
    public Servo servoDepositL;

    //Sensors
    public DistanceSensor sensorDistanceR;
    public DistanceSensor sensorDistanceL;
    public ColorSensor sensorColorR;
    public ColorSensor sensorColorL;

   public void init(HardwareMap ahwmap, Telemetry myTelemetry) {
        super.init(ahwmap, myTelemetry);
        mtrLiftL = ahwmap.dcMotor.get("mtrLiftL");
        mtrLiftR = ahwmap.dcMotor.get("mtrLiftR");

        mtrLiftR.setDirection(DcMotorSimple.Direction.REVERSE);

        servoDepositR = ahwmap.servo.get("servoDepositR");
        servoDepositL = ahwmap.servo.get("servoDepositL");



        sensorColorR = ahwmap.colorSensor.get("servoDepositR");
        sensorColorL = ahwmap.colorSensor.get("servoDepositL");
        sensorDistanceR = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorColorR");
        sensorDistanceL = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorColorL");
    }

    public void reset() {
        mtrLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

