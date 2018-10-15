package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

            // move forward and use Vuforia to determine which of the minerals is gld.

            if // gold is in middle
            {
                // make necessary motions to orient flicker with the mineral (exact motions/encoder values TBH)
                // flick gold (motor for this not installed yet)
                // move diagonally until we hit the wall
                // move towards the depot (find encoder value)
                // move the opposite direction towards the crater and onto it (find encoder value)
            }

            if // gold is to left
            {
                // make necessary motions to orient flicker with the mineral (exact motions/encoder values TBD)
                // flick gold (motor for this not installed yet)
                // move diagonally until we hit the wall
                // move towards the depot (find encoder value)
                // move the opposite direction towards the crater and onto it (find encoder value)
            }

            if // gold is to right
            {
                // make necessary motions to orient flicker with the mineral (exact motions/encoder values TBD)
                // flick gold (motor for this not installed yet)
                // move diagonally until we hit the wall
                // move towards the depot (find encoder value)
                // move the opposite direction towards the crater and onto it (find encoder value)
            }

            finished = true;
        }
    }
}
