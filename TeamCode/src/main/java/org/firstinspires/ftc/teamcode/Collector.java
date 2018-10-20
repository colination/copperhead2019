package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class    Collector extends robotPart {
    public Servo srvCollectorL;
    public Servo srvCollectorR;

    public void init(HardwareMap ahwmap, Telemetry myTelemetry){
        super.init(ahwmap, myTelemetry);
        srvCollectorL = ahwmap.servo.get("srvCollectorL");
        srvCollectorR = ahwmap.servo.get("srvCollectorR");
    }
}
