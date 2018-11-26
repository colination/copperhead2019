package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CopperHeadRobot {
    DriveTrain driveTrain = new DriveTrain();
    LiftAndHook liftAndHook = new LiftAndHook();
    Collector collector = new Collector();

    //gyroTestAuto gyro = new gyroTestAuto();
    public void init(HardwareMap ahwmap, Telemetry myTelemetry, boolean enable){
        liftAndHook.init(ahwmap, myTelemetry);
        driveTrain.init(ahwmap, myTelemetry, enable);
        collector.init(ahwmap, myTelemetry);

      //  gyro.init(ahwmap, myTelemetry);

        //gyro.init(ahwmap, myTelemetry);

    }
    public void markerAndPark(double power){
        driveTrain.setSideRoller(1);
        driveTrain.goLean(12,power,6,true);
        //collector.depositMarker();
        driveTrain.goLean(-60,power,24,false);
    }

}
