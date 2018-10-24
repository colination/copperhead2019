package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// NOTE: THIS AUTO IS ONLY FOR THE RED POSITION CLOSEST TO THE DEPOT

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

            // orient robot ? How much will the robot shift upon landing from its initial orientation? We need hook finished to test this.

            // Use OpenCD to determine which of the minerals is the gold.

            if // gold is in middle
            {
                // turn 90 degrees (need gyro)
                // robot.collector.Extend(.3);
                // move diagonally until we hit the wall
                // robot.driveTrain.goInches(50, .5, 6); << VALUES ARE ROUGH
                // robot.driveTrain.goInches(-70, .6, 6); << VALUES ARE ROUGH
            }

            if // gold is to left
            {
                // turn 45 degrees (need gyro)
                // robot.collector.Extend(.3);
                // move diagonally until we hit the wall
                // robot.driveTrain.goInches(50, .5, 6); << VALUES ARE ROUGH
                // robot.driveTrain.goInches(-70, .6, 6); << VALUES ARE ROUGH
            }

            if // gold is to right
            {
                // turn 135 degrees (need gyro)
                // robot.collector.Extend(.3);
                // move diagonally until we hit the wall
                // robot.driveTrain.goInches(50, .5, 6); << VALUES ARE ROUGH
                // robot.driveTrain.goInches(-70, .6, 6); << VALUES ARE ROUGH
            }

            finished = true;
        }
    }
}
