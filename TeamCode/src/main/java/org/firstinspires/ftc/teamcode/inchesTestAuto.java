package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Inch Test", group = "12596")
public class inchesTestAuto extends LinearOpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap,telemetry);
        telemetry.addData("ticks", robot.driveTrain.mtrBL.getCurrentPosition());
        telemetry.update();
        boolean finished = false;
        waitForStart();
        while (opModeIsActive() && finished == false){
            robot.driveTrain.goInches(12,.5,6);
            robot.driveTrain.goInches(24.0, .2, 6);
            robot.driveTrain.stopMotors();
            finished = true;
        }

    }

}
