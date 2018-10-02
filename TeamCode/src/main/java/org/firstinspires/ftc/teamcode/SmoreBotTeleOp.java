package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    }
}
