package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class    Collector extends robotPart {
    public Servo srvCollectorL;
    public Servo srvCollectorR;
    public Servo srvFlopL;
    public Servo srvFlopR;

    public DcMotor mtrExtendL;
    public DcMotor mtrExtendR;

    public void init(HardwareMap ahwmap, Telemetry myTelemetry){
        super.init(ahwmap, myTelemetry);

        srvCollectorL = ahwmap.servo.get("srvCollectorL");
        srvCollectorR = ahwmap.servo.get("srvCollectorR");
        srvFlopL = ahwmap.servo.get("srvFlopL");
        srvFlopR = ahwmap.servo.get("srvFlopR");

        mtrExtendL = ahwmap.dcMotor.get("mtrExtenderL");
        mtrExtendR = ahwmap.dcMotor.get("mtrExtenderR");
        mtrExtendR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
