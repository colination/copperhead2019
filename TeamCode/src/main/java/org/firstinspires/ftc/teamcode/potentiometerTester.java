package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "potemtiometer", group = "12596")
public class potentiometerTester extends LinearOpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry,false);
        if (opModeIsActive()){
            telemetry.addData("potentiometer value",robot.liftAndHook.angleTurned(robot.liftAndHook.potentiometer));
            telemetry.update();
        }
    }
}
