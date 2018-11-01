package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// NOTE: THIS AUTO IS ONLY FOR THE POSITION CLOSEST TO THE DEPOT



/*
>>>>>>> b8c084d4f73b8b98b5eb5a880ca7c9ee8c7fc39a
>>>>>>> d0e3923112c461bd993a41a4569e774d5714fe6a
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
 v
            // unhook from lander -- lift will have to move a certain encoder value (see Joel's code for tele-op lift when finished)
            robot.driveTrain.srvRoller.setPosition(1);
            // turn to the left, begin scanning while turning
            // store the values of each of the minerals as an array, base if statements on that

            if  // gold is in middle
            {
                // turn to face middle mineral (exact angle TBD)
                robot.collector.Extend(.3); // extend collector to hit gold
                robot.collector.Retract(.3); // retract collector
                // turn back the same angle as above but in the opposite direction
                robot.driveTrain.goInches(60, 0.6, 8); // move backwards until we hit wall
                // turn right to orient ourselves with the wall
                robot.driveTrain.goInches(50, .6, 8); // move into depot
                robot.liftAndHook.servoDepositR.setPosition(.85); // angle deposit down to drop the marker into the depot
                robot.liftAndHook.servoDepositR.setPosition(0); // move the deposit back up to the 0 position
                robot.driveTrain.goInches(-70, .6, 8); // move onto crater
            }

            if  // gold is to left
            {
                // turn to face left mineral (exact angle TBD)
                robot.collector.Extend(.3); // extend collector to hit gold
                robot.collector.Retract(.3); // retract collector
                // turn back the same angle as above but in the opposite direction
                robot.driveTrain.goInches(60, 0.6, 8); // move backwards until we hit wall
                // turn right to orient ourselves with the wall
                robot.driveTrain.goInches(50, .6, 8); // << move into depot
                robot.liftAndHook.servoDepositR.setPosition(.85); // angle deposit down to drop the marker into the depot
                robot.liftAndHook.servoDepositR.setPosition(0); // move the deposit back up to the 0 position
                robot.driveTrain.goInches(-70, .6, 8); // move onto crater
            }

            if  // gold is to right
            {
                // turn to face right mineral(exact angle TBD)
                robot.collector.Extend(.3); // extend collector to hit gold
                robot.collector.Retract(.3); // retract collector
                // turn back the same angle as above but in the opposite direction
                robot.driveTrain.goInches(60, 0.6, 8); // move backwards until we hit wall
                // turn right to orient ourselves with the wall
                robot.driveTrain.goInches(50, .6, 8); // move into depot
                robot.liftAndHook.servoDepositR.setPosition(.85); // angle deposit down to drop the marker into the depot
                robot.liftAndHook.servoDepositR.setPosition(0); // move the deposit back up to the 0 position
                robot.driveTrain.goInches(-70, .6, 8?); // move onto crater
            }

            finished = true;
        }
    }
}

*/