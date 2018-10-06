package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "TankTele",group = "12596")
public class TankBotTeleOp extends OpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    @Override
    public void init() {
        robot.init(hardwareMap,telemetry);
        telemetry.addData("Hello","Driver");
        telemetry.update();
    }

    @Override
    public void loop() {
        double leftPower  = -gamepad1.left_stick_y ;
        double rightPower = -gamepad1.right_stick_y ;

        robot.driveTrain.Tank(leftPower, rightPower); // Tank Drive

        /*color sorted teleop, use once color sensors are wired
        // Sets deposits straight up
        if (gamepad1.y) {
            // move to 0 degrees.
            robot.liftAndHook.servoL.setPosition(0);
            robot.liftAndHook.servoR.setPosition(1);
        }

        // Left side deposit
        if (robot.liftAndHook.sensorDistanceL.getDistance(DistanceUnit.CM) < 7.0) {
            if (gamepad1.a) {
                if (robot.liftAndHook.sensorColorL.blue() > 70) {
                    robot.liftAndHook.servoL.setPosition(.66); // Deposit silver
                }
                else {
                    robot.liftAndHook.servoL.setPosition(.40); // Deposit Gold
                }
            }
        }
        // Right side deposit
        if (robot.liftAndHook.sensorDistanceR.getDistance(DistanceUnit.CM) < 7.0)  {
            if (gamepad1.a) {
                if (robot.liftAndHook.sensorColorR.blue() > 70) { // Deposit silver mineral
                    robot.liftAndHook.servoR.setPosition(0);
                }
                else {
                    robot.liftAndHook.servoR.setPosition(.40); // Deposit Gold mineral
                }
            }
        }*/
        if (gamepad1.y) {
            // move to 0 degrees.
            robot.liftAndHook.servoL.setPosition(0);
            robot.liftAndHook.servoR.setPosition(1);
        }
        if (gamepad1.b) {
            // Deposit Gold
            robot.liftAndHook.servoL.setPosition(.4);
            robot.liftAndHook.servoR.setPosition(.37);
        }
        if ( gamepad1.x) {
            robot.liftAndHook.servoL.setPosition(.66);
            robot.liftAndHook.servoR.setPosition(.37);
        }
        if (gamepad1.a) {
            // Deposit silver
            robot.liftAndHook.servoL.setPosition(.66);
            robot.liftAndHook.servoR.setPosition(0);
        }


        //Telemetry
        telemetry.addData("Distance (cm)",
                String.format(Locale.US, "%.02f", robot.liftAndHook.sensorDistanceR.getDistance(DistanceUnit.CM)));
        telemetry.update();

    }
}

