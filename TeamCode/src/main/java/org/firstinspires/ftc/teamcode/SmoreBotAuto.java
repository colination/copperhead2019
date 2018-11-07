//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//// NOTE: THIS AUTO IS ONLY FOR THE POSITION CLOSEST TO THE DEPOT
//
//@Disabled
//@Autonomous(name = "SmoreBot Auto", group = "12596")
//public class SmoreBotAuto extends LinearOpMode {
//    CopperHeadRobot robot = new CopperHeadRobot();
//    @Override
//    public void runOpMode() throws InterruptedException {
//        robot.init(hardwareMap,telemetry);
//        boolean finished = false;
//        waitForStart();
//        while (opModeIsActive() && finished == false){
//
//            // unhook from lander -- lift will have to move a certain encoder value (see Joel's code for tele-op lift when finished)
//            robot.driveTrain.srvRoller.setPosition(1);
//            boolean middle = false;
//            boolean left = false;
//            boolean right = false;
//
//            if (middle == true)
//            {
//                robot.gyro.rotate(90, .25); // rotate to face gold
//                robot.collector.Extend(.3); // extend collector to hit gold
//                sleep(200); // wait .2 seconds
//                robot.collector.Retract(.3); // retract collector
//                robot.gyro.rotate(-90, .25); // rotate back to be horizontal with the lander
//                robot.driveTrain.goInches(60, 0.6, 6); // move backwards until we hit wall
//                robot.gyro.rotate(45, .25); // rotate right to line up with wall
//                robot.driveTrain.goInches(50, .6, 6); // move into depot
//                robot.collector.depositMarker(); // deposit marker
//                robot.driveTrain.goInches(-70, .6, 6); // move onto crater
//            }
//
//            if  (left == true)
//            {
//                robot.gyro.rotate(45, .25); // rotate to face gold
//                robot.collector.Extend(.3); // extend collector to hit gold
//                sleep(200); // wait .2 seconds
//                robot.collector.Retract(.3); // retract collector
//                robot.gyro.rotate(-45, .25); // rotate to be horizontal with the lander
//                robot.driveTrain.goInches(60, 0.6, 6); // move backwards until we hit wall
//                robot.gyro.rotate(45, .25); // rotate to line up with wall
//                robot.driveTrain.goInches(50, .6, 6); // move into depot
//                robot.collector.depositMarker(); // deposit marker
//                robot.driveTrain.goInches(-70, .6, 6); // move onto crater
//            }
//
//            if (right == true)
//            {
//                robot.gyro.rotate(135, .25); // rotate to face gold
//                robot.collector.Extend(.3); // extend collector to hit gold
//                sleep(200); // wait .2 seconds
//                robot.collector.Retract(.3); // retract collector
//                robot.gyro.rotate(-135, .25); // rotate back to be horizontal with lander
//                robot.driveTrain.goInches(60, 0.6, 6); // move backwards until we hit wall
//                robot.gyro.rotate(45, .25); // rotate to line up with wall
//                robot.driveTrain.goInches(50, .6, 6); // move into depot
//                robot.collector.depositMarker(); // deposit marker
//                robot.driveTrain.goInches(-70, .6, 6); // move onto crater
//            }
//
//            finished = true;
//        }
//    }
//}
