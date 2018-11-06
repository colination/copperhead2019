package org.firstinspires.ftc.teamcode;
// This class is where we put methods that all classes use.
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class robotPart {
    protected Telemetry privateTelemetry;
    protected LinearOpMode privateOpMode;
    public ElapsedTime runtime;

    public void init(HardwareMap ahwmap, Telemetry myTelemetry){
        privateTelemetry = myTelemetry;
        privateTelemetry.update();
    }
}
