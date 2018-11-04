package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CopperHeadRobot {
    DriveTrain driveTrain = new DriveTrain();
    LiftAndHook liftAndHook = new LiftAndHook();
    Collector collector = new Collector();
    //gyroTestAuto gyro = new gyroTestAuto();

    public void init(HardwareMap ahwmap, Telemetry myTelemetry){
        driveTrain.init(ahwmap, myTelemetry);
        liftAndHook.init(ahwmap, myTelemetry);
        collector.init(ahwmap, myTelemetry);
<<<<<<< HEAD
      //  gyro.init(ahwmap, myTelemetry);
=======
        //gyro.init(ahwmap, myTelemetry);
>>>>>>> 646b7a70591a81216aab4aed544bbadfef6685c3
    }
    public void markerAndPark(double power){
        driveTrain.goLean(12,power,6,true);
        collector.depositMarker();
        driveTrain.goLean(-60,power,24,false);
    }

}
