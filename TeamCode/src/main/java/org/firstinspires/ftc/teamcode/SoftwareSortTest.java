package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "SWSort",group = "12596")
public class SoftwareSortTest extends OpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    float hsvLValues[] = {0F, 0F, 0F};
    float hsvRValues[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;
    double position;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry, false); // this is false here?
        telemetry.addData("Hello", "Driver");
        telemetry.addData("Yeet","yeet");
        telemetry.update();
        robot.liftAndHook.srvShift.setPosition(.94);

    }

    @Override
    public void loop() {

        // Sets deposits straight up
        if (gamepad2.y) {
            // move to 0 degrees.
            robot.collector.servoDepositL.setPosition(0);
            robot.collector.servoDepositR.setPosition(1);
        }
        // Sets deposits straight down
        if (gamepad2.b) {
            // move to 0 degrees.
            robot.collector.servoDepositL.setPosition(.37);
            robot.collector.servoDepositR.setPosition(.53);
        }
        // Sets deposits at angle
        if (gamepad2.x) {
            // move to 0 degrees.
            robot.collector.servoDepositL.setPosition(.21);
            robot.collector.servoDepositR.setPosition(.73);
        }
        Color.RGBToHSV((int) (robot.collector.sensorColorL.red() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorL.green() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorL.blue() * SCALE_FACTOR),
                hsvLValues);
        Color.RGBToHSV((int) (robot.collector.sensorColorR.red() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorR.green() * SCALE_FACTOR),
                (int) (robot.collector.sensorColorR.blue() * SCALE_FACTOR),
                hsvRValues);

/*******************************************************************
        // Left side deposit
        if (robot.liftAndHook.sensorDistanceL.getDistance(DistanceUnit.CM) < 7.0) {
            if (gamepad2.a) {
                if (robot.liftAndHook.sensorColorL.blue() > 100) {
                    robot.liftAndHook.servoDepositL.setPosition(.63); // Deposit silver
                } else {
                    robot.liftAndHook.servoDepositL.setPosition(.40); // Deposit Gold
                }
            }
        }

        // Right side deposit
        if (robot.liftAndHook.sensorDistanceR.getDistance(DistanceUnit.CM) < 7.0) {
            if (gamepad2.a) {
                if (robot.liftAndHook.sensorColorR.blue() > 100) { // Deposit silver mineral
                    robot.liftAndHook.servoDepositR.setPosition(0);
                } else {
                    robot.liftAndHook.servoDepositR.setPosition(.33); // Deposit Gold mineral
                }
            }
        }
 **********************************************************************/
        // Left side deposit
        if (gamepad2.a) {
            if (hsvRValues[0] > 100) {
                robot.collector.servoDepositL.setPosition(.62); // Deposit silver
            } else {
                robot.collector.servoDepositL.setPosition(.4); // Deposit Gold
            }
        }
        // Right side deposit
        if (gamepad2.a) {
            if (hsvLValues[0] > 100) { // Deposit silver mineral
                robot.collector.servoDepositR.setPosition(.35);
            } else {
                robot.collector.servoDepositR.setPosition(.57); // Deposit Gold mineral
            }
        }
        if (gamepad1.y) {
            position = .9;
        }
        if (gamepad2.dpad_up) {
            position = .94;
        }
        if (gamepad2.dpad_down) {
            position = .84;
        }


        //Telemetry
        telemetry.addData("rightDistance (cm)",
                String.format(Locale.US, "%.02f", robot.collector.sensorDistanceR.getDistance(DistanceUnit.CM)));
        telemetry.addData("leftDistance (cm)",
                String.format(Locale.US, "%.02f", robot.collector.sensorDistanceL.getDistance(DistanceUnit.CM)));
        //telemetry.addData("rightBlue ", robot.collector.sensorColorR.blue());
        //telemetry.addData("leftBlue ", robot.collector.sensorColorL.blue());
        telemetry.addData("LHue", hsvLValues[0]);
        telemetry.addData("RHue", hsvRValues[0]);
    }
}


