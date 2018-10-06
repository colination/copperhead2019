package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftAndHook extends robotPart {
    //Motors
    public DcMotor mtrLiftR;
    public DcMotor mtrLiftL;

    //Servos
    public Servo servoR;
    public Servo servoL;
    //Sensors
    public DistanceSensor sensorDistanceR;
    public DistanceSensor sensorDistanceL;
    public ColorSensor sensorColorR;
    public ColorSensor sensorColorL;

    public void init(HardwareMap ahwmap, Telemetry myTelemetry) {
        super.init(ahwmap, myTelemetry);
        mtrLiftL = ahwmap.dcMotor.get("mtrLiftL");
        mtrLiftR = ahwmap.dcMotor.get("mtrLiftR");
        servoR = ahwmap.servo.get("servoR");
        servoL = ahwmap.servo.get("servoL");
        sensorColorR = ahwmap.colorSensor.get("sensorColor");
        sensorColorL = ahwmap.colorSensor.get("sensorColor");
        sensorDistanceR = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorDistance");
        sensorDistanceL = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorDistance");
    }
}
