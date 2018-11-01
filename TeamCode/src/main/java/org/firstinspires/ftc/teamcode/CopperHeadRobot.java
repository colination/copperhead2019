package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CopperHeadRobot {
    DriveTrain driveTrain = new DriveTrain();
    LiftAndHook liftAndHook = new LiftAndHook();
    Collector collector = new Collector();

    public void init(HardwareMap ahwmap, Telemetry myTelemetry){
        driveTrain.init(ahwmap, myTelemetry);
        liftAndHook.init(ahwmap, myTelemetry);
        collector.init(ahwmap, myTelemetry);
    }
    public void markerAndPark(double power){

    }

}
