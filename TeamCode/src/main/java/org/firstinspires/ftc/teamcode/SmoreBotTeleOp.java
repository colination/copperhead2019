package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "SmoreTele",group = "12596")
public class SmoreBotTeleOp extends OpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    @Override
    public void init() {
        robot.init(hardwareMap,telemetry);
        telemetry.addData("Hello","Driver");
        telemetry.update();
    }

    @Override
    public void loop() {
        double FwdBack = -gamepad1.right_stick_y;
        double Turn = -gamepad1.left_stick_x;

        if (Math.abs(FwdBack) > 0.1){
            robot.driveTrain.move(FwdBack);
        }
        if (Math.abs(Turn) > 0.1){
            robot.driveTrain.Turn(Turn);
        }

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
                    robot.liftAndHook.servoL.setPosition(.66);
                }
                else {
                    robot.liftAndHook.servoL.setPosition(.40);
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
        }
    }
}

