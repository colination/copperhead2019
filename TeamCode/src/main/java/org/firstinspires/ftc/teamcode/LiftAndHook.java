package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftAndHook extends robotPart {

    public Servo servoR;
    public Servo servoL;
    public DistanceSensor sensorDistanceR;
    public DistanceSensor sensorDistanceL;
    public ColorSensor sensorColorR;
    public ColorSensor sensorColorL;

    public void init(HardwareMap ahwmap, Telemetry myTelemetry) {
        super.init(ahwmap, myTelemetry);
        servoR = ahwmap.servo.get("servoR");
        servoL = ahwmap.servo.get("servoL");
        sensorColorR = ahwmap.colorSensor.get("sensorColor");
        sensorColorL = ahwmap.colorSensor.get("sensorColor");
        sensorDistanceR = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorDistance");
        sensorDistanceL = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorDistance");
    }



}
