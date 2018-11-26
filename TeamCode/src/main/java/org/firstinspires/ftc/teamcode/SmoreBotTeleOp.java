package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name = "Smore TeleOp",group = "12596")
public class SmoreBotTeleOp extends OpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    @Override
    public void init() {
        robot.init(hardwareMap,telemetry,false);
        telemetry.addData("Hello","Driver");
        telemetry.update();
    }
    public double WeightAvg(double x, double y){
        double speed_D = 0;
        if (Math.abs(x) + Math.abs(y) !=0 ){
            speed_D = (((x * Math.abs(x)) + (y * Math.abs(y))) /(Math.abs(x) + Math.abs(y)));
        }
        return(speed_D);
    }
    @Override
    public void loop() {
        double FwdBack = -gamepad1.right_stick_y;
        double Turn = -gamepad1.left_stick_x;
        double Lift = -gamepad2.right_stick_y;

        if (Math.abs(FwdBack) > 0.1){
            FwdBack = FwdBack;
        }
        else {
            FwdBack = 0;
        }
        if (Math.abs(Turn) > 0.1){
            Turn = Turn;
        }
        else {
            Turn = 0;
        }
        if (Math.abs(Lift) > 0.1){
            Lift = Lift;
        }
        else {
            Lift = 0;
        }
        robot.driveTrain.mtrFL.setPower(WeightAvg(FwdBack, Turn));
        robot.driveTrain.mtrBL.setPower(WeightAvg(FwdBack, Turn));
        robot.driveTrain.mtrFR.setPower(WeightAvg(FwdBack, -Turn));
        robot.driveTrain.mtrBR.setPower(WeightAvg(FwdBack, -Turn));

        robot.liftAndHook.mtrLiftR.setPower(Lift);
        robot.liftAndHook.mtrLiftL.setPower(Lift);
        // Sets deposits straight up
        if (gamepad1.y) {
            // move to 0 degrees.
            robot.collector.servoDepositL.setPosition(0);
            robot.collector.servoDepositR.setPosition(1);
        }
            if (robot.collector.sensorDistanceL.getDistance(DistanceUnit.CM) < 7.0) {
                if (gamepad1.a) {
                    if (robot.collector.sensorColorL.blue() > 70) {
                        robot.collector.servoDepositL.setPosition(.66);
                    } else {
                        robot.collector.servoDepositL.setPosition(.40);
                    }
                }
            }
        // Right side deposit
        if (robot.collector.sensorDistanceR.getDistance(DistanceUnit.CM) < 7.0)  {
            if (gamepad1.a) {
                if (robot.collector.sensorColorR.blue() > 70) { // Deposit silver mineral
                    robot.collector.servoDepositR.setPosition(0);
                }
                else {
                    robot.collector.servoDepositR.setPosition(.40); // Deposit Gold mineral
                }
            }
        }
    }
}