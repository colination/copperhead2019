package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class robotPart {
    protected Telemetry privateTelemetry;
    protected LinearOpMode privateOpMode;
    private ElapsedTime runtime;

    public void init(HardwareMap ahwmap, Telemetry myTelemetry, LinearOpMode myOpMode){

    }
}
