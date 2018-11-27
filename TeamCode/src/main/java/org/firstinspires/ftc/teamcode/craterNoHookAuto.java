package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name = "CraterNoHookAuto", group = "12596")

public class craterNoHookAuto extends LinearOpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine().addData("literally just goes straight pray to god we don't have to use this",robot.driveTrain.mtrBL.getCurrentPosition());
        telemetry.addLine().addData("goes straight",robot.driveTrain.mtrBL.getCurrentPosition());
        robot.init(hardwareMap,telemetry,true);
        robot.liftAndHook.mtrLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftAndHook.mtrLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("ticks", robot.driveTrain.mtrBL.getCurrentPosition());
        telemetry.update();
        boolean finished = false;
        waitForStart();
        if (opModeIsActive() && finished == false){
            robot.driveTrain.setSideRoller(1);
            robot.driveTrain.goInches(21,.5,12); // literally just goes straight pray to god we don't have to use this
            robot.driveTrain.stopMotors();
            finished = true;
        }
    }

    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        robot.driveTrain.resetAngle();
        telemetry.addLine().addData("Robot Angle", robot.driveTrain.getAngle());

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.driveTrain.mtrFL.setPower(leftPower);
        robot.driveTrain.mtrBL.setPower(leftPower);
        robot.driveTrain.mtrFR.setPower(rightPower);
        robot.driveTrain.mtrBR.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.driveTrain.getAngle() == 0) {}

            while (opModeIsActive() && robot.driveTrain.getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && robot.driveTrain.getAngle() < degrees) {}

        // turn the motors off.
        robot.driveTrain.stopMotors();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        robot.driveTrain.resetAngle();
    }
}