package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "Deposit Fix",group = "12596")
public class depositFix extends OpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    double position = 0;
    double uprightL = 0;
    double uprightR = .88;
    double goldR = .33;
    double goldL = .40;
    double silverL = .66;
    double silverR = .9;
    @Override
    public void init() {
        robot.init(hardwareMap,telemetry,false);
        telemetry.addData("Hello","Driver");
        telemetry.update();
        robot.liftAndHook.reset();
        robot.liftAndHook.mtrLiftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftAndHook.mtrLiftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_down) {
            position -= .02;
        }
        if (gamepad2.dpad_up) {
            position += .02;
        }
        //silver
        if(gamepad2.a) {
            robot.collector.servoDepositR.setPosition(0 + position);
            robot.collector.servoDepositL.setPosition(.66 + position);
        }
        // gold
        if (gamepad2.b) {
            robot.collector.servoDepositR.setPosition(.33 + position);
            robot.collector.servoDepositL.setPosition(.40 + position);
        }
        //upright
        if (gamepad2.y) {
            robot.collector.servoDepositR.setPosition(.88 + position);
            robot.collector.servoDepositL.setPosition(0 + position);
        }
        if (gamepad2.x) {
            position = 0;
        }

        //Telemetry
        telemetry.addData("position", position);
        telemetry.update();
    }
}

