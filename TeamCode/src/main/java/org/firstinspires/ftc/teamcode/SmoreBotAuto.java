package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// NOTE: THIS AUTO IS ONLY FOR THE RED POSITION CLOSEST TO THE DEPOT




/*
>>>>>>> b8c084d4f73b8b98b5eb5a880ca7c9ee8c7fc39a
<<<<<<< HEAD
=======
>>>>>>> d0e3923112c461bd993a41a4569e774d5714fe6a
=======
>>>>>>> 078deb2c21e450cecddb9fb357373b4ab30d8a12
>>>>>>> 66396390c60a4972bce190cc5ede6e10f11451d4
>>>>>>> da305ae6d0ba7c78a194da6abae36edf51bfd7d6
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
<<<<<<< HEAD

            // orient robot ? How much will the robot shift upon landing from its initial orientation? We need hook finished to test this.

            // Use OpenCD to determine which of the minerals is the gold.

            //if // gold is in middle
            {
                // turn 90 degrees (need gyro)
                // robot.collector.Extend(.3);
                // move diagonally until we hit the wall
                // robot.driveTrain.goInches(50, .5, 6); << VALUES ARE ROUGH
                // robot.driveTrain.goInches(-70, .6, 6); << VALUES ARE ROUGH
=======
            robot.driveTrain.srvRoller.setPosition(1);
            robot.gyro.rotate(15, -.5); // turn left to scan leftmost mineral
            robot.gyro.rotate(15, .5); // turn back to scan middle mineral
            robot.gyro.rotate(15, .5); // turn right to scan rightmost mineral
            // store the values of each of the minerals as an array, base if statements on that
            if  // gold is in middle
            {
                robot.gyro.rotate(15, -.5); // rotate back to face gold
                robot.collector.Extend(.3); // extend collector to hit gold
                sleep(200); // wait .2 seconds
                robot.collector.Retract(.3); // retract collector
                // already horizontal with lander
                robot.driveTrain.goInches(60, 0.6, 8); // move backwards until we hit wall
                robot.gyro.rotate(30, .5);// turn right to orient ourselves with the wall
                robot.driveTrain.goInches(50, .6, 8); // move into depot
                robot.collector.depositMarker(); // deposit marker
                robot.driveTrain.goInches(70, -.6, 8); // move onto crater
>>>>>>> da305ae6d0ba7c78a194da6abae36edf51bfd7d6
            }

            //if // gold is to left
            {
<<<<<<< HEAD
                // turn 45 degrees (need gyro)
                // robot.collector.Extend(.3);
                // move diagonally until we hit the wall
                // robot.driveTrain.goInches(50, .5, 6); << VALUES ARE ROUGH
                // robot.driveTrain.goInches(-70, .6, 6); << VALUES ARE ROUGH
            }

            //if // gold is to right
            {
                // turn 135 degrees (need gyro)
                // robot.collector.Extend(.3);
                // move diagonally until we hit the wall
                // robot.driveTrain.goInches(50, .5, 6); << VALUES ARE ROUGH
                // robot.driveTrain.goInches(-70, .6, 6); << VALUES ARE ROUGH
=======
                robot.gyro.rotate(30, -.5); // rotate back to face gold
                robot.collector.Extend(.3); // extend collector to hit gold
                sleep(200); // wait .2 seconds
                robot.collector.Retract(.3); // retract collector
                robot.gyro.rotate(15, 5); // rotate
                robot.driveTrain.goInches(60, 0.6, 8); // move backwards until we hit wall
                robot.gyro.rotate(30, .5);// turn right to orient ourselves with the wall
                robot.driveTrain.goInches(50, .6, 8); // << move into depot
                robot.collector.depositMarker(); // deposit marker
                robot.driveTrain.goInches(70, -.6, 8); // move onto crater
            }

            if // gold is to right
            {
                // stay put since already facing right most
                robot.collector.Extend(.3); // extend collector to hit gold
                sleep(200); // wait .2 seconds
                robot.collector.Retract(.3); // retract collector
                // turn back the same angle as above but in the opposite direction
                robot.driveTrain.goInches(60, 0.6, 8); // move backwards until we hit wall
                robot.gyro.rotate(30, .5); // turn right to orient ourselves with the wall
                robot.driveTrain.goInches(50, .6, 8); // move into depot
                robot.collector.depositMarker(); // deposit marker
                robot.driveTrain.goInches(70, -.6, 8?); // move onto crater
>>>>>>> da305ae6d0ba7c78a194da6abae36edf51bfd7d6
            }

            finished = true;
        }
    }
}
*/