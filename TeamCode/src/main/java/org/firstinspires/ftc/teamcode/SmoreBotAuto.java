package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// NOTE: THIS AUTO IS ONLY FOR THE POSITION CLOSEST TO THE DEPOT

@Disabled
@Autonomous(name = "SmoreBot Auto", group = "12596")
public class SmoreBotAuto extends LinearOpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        boolean finished = false;
        waitForStart();
        while (opModeIsActive() && finished == false){

            // unhook from lander -- lift will have to move a certain encoder value (see Joel's code for tele-op lift when finished)
            robot.driveTrain.srvRoller.setPosition(1);
            boolean middle = false;
            boolean left = false;
            boolean right = false;

            if (middle == true)
            {
                robot.gyro.rotate(15, -.5); // rotate back to face gold
                robot.collector.Extend(.3); // extend collector to hit gold
                sleep(200); // wait .2 seconds
                robot.collector.Retract(.3); // retract collector
                // already horizontal with lander
                robot.driveTrain.goInches(60, 0.6, 8); // move backwards until we hit wall
                // turn right to orient ourselves with the wall
                robot.driveTrain.goInches(50, .6, 8); // move into depot
                robot.collector.depositMarker(); // deposit marker
                robot.driveTrain.goInches(70, -.6, 8); // move onto crater
            }

            if  (left == true)
            {
                robot.gyro.rotate(30, -.5); // rotate back to face gold
                robot.collector.Extend(.3); // extend collector to hit gold
                sleep(200); // wait .2 seconds
                robot.collector.Retract(.3); // retract collector
                robot.gyro.rotate(15, 5); // rotate
                robot.driveTrain.goInches(60, 0.6, 8); // move backwards until we hit wall
                // turn right to orient ourselves with the wall
                robot.driveTrain.goInches(50, .6, 8); // << move into depot
                robot.collector.depositMarker(); // deposit marker
                robot.driveTrain.goInches(70, -.6, 8); // move onto crater
            }

            if (right == true)
            {
                // stay put since already facing right most
                robot.collector.Extend(.3); // extend collector to hit gold
                sleep(200); // wait .2 seconds
                robot.collector.Retract(.3); // retract collector
                // turn back the same angle as above but in the opposite direction
                robot.driveTrain.goInches(60, 0.6, 8); // move backwards until we hit wall
                // turn right to orient ourselves with the wall
                robot.driveTrain.goInches(50, .6, 8); // move into depot
                robot.collector.depositMarker(); // deposit marker
                robot.driveTrain.goInches(70, -.6, 8?); // move onto crater
            }

            finished = true;
        }
    }
}
