package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

<<<<<<< HEAD
@Autonomous(name = "Depot Hook", group = "12596")
=======
@Autonomous(name = "depotHook", group = "12596")
>>>>>>> 91b16c4f952570441ac88f78c7a535a06446e293
public class depotHookAuto extends LinearOpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    DigitalChannel touch;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;


    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);

        //robot.driveTrain.mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.driveTrain.mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.driveTrain.mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.driveTrain.mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // get a reference to REV Touch sensor.
        //touch = hardwareMap.digitalChannel.get("touch_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        //telemetry.addData("Mode", "running");
        //telemetry.update();

        sleep(1000);

        // drive until end of period.
        boolean finished = false;

        while (opModeIsActive() && finished == false)
        {
            // Use gyro to drive in a straight line.
            correction = checkDirection();
            robot.liftAndHook.goInches(11.5, .8, 6);
            sleep(2000);
            robot.driveTrain.goInches(-2, .2, 4);
            sleep(2000);
            robot.driveTrain.setSideRoller(.4);
            robot.liftAndHook.goInches(-11.5, .8, 6);
            sleep(2000);
            rotate(-90, .25);
            robot.driveTrain.stopMotors();



            //telemetry.addLine().addData("1 imu heading", lastAngles.firstAngle);
            //telemetry.addLine().addData("2 global heading", globalAngle);
            //telemetry.addLine().addData("3 correction", correction);


            telemetry.addLine().addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addLine().addData("2 global heading", globalAngle);
            telemetry.addLine().addData("3 correction", correction);
            //telemetry.addLine().addData("Robot Angle", getAngle());
            //telemetry.update();
            //robot.driveTrain.mtrFL.setPower(-power + correction);
            //robot.driveTrain.mtrBL.setPower(-power + correction);
            //robot.driveTrain.mtrFR.setPower(-power);
            //robot.driveTrain.mtrBR.setPower(-power);

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.
            finished = true;
        }

        // turn the motors off.
        robot.driveTrain.stopMotors();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        robot.driveTrain.turnMode();
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();
        telemetry.addLine().addData("Robot Angle", getAngle());

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees > 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees < 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
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
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {
                robot.driveTrain.mtrFL.setPower(leftPower);
                robot.driveTrain.mtrBL.setPower(leftPower);
                robot.driveTrain.mtrFR.setPower(rightPower);
                robot.driveTrain.mtrBR.setPower(rightPower);
                telemetry.addData("degrees", getAngle());
                telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                robot.driveTrain.mtrFL.setPower(leftPower);
                robot.driveTrain.mtrBL.setPower(leftPower);
                robot.driveTrain.mtrFR.setPower(rightPower);
                robot.driveTrain.mtrBR.setPower(rightPower);
                telemetry.addData("degrees", getAngle());
                telemetry.update();
            }

        // turn the motors off.
        robot.driveTrain.stopMotors();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    public void encoderDrive(double Lspeed, double Rspeed, double Inches, double rampup) throws InterruptedException {

        //double     COUNTS_PER_MOTOR_REV    = 560 ;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
        //double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is the ratio between the motor axle and the wheel
        //double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
        //double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
        //(WHEEL_DIAMETER_INCHES * 3.1415);
        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;
        // reset the timeout time and start motion.
        robot.driveTrain.runtime.reset();
        robot.driveTrain.reset();
        // Ensure that the opmode is still active
        while (opModeIsActive()) {
            // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
            newLeftTarget = (robot.driveTrain.mtrFL.getCurrentPosition() + robot.driveTrain.mtrBL.getCurrentPosition()) / 2 + (int) (Inches * robot.driveTrain.COUNTS_PER_INCH);
            newRightTarget = (robot.driveTrain.mtrFR.getCurrentPosition() + robot.driveTrain.mtrBR.getCurrentPosition()) / 2 + (int) (Inches * robot.driveTrain.COUNTS_PER_INCH);

            // set run mode
            robot.driveTrain.mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.driveTrain.mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.driveTrain.mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.driveTrain.mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
            while ((Math.abs(robot.driveTrain.mtrFL.getCurrentPosition() + robot.driveTrain.mtrBL.getCurrentPosition()) / 2 < newLeftTarget &&
                    Math.abs(robot.driveTrain.mtrFR.getCurrentPosition() + robot.driveTrain.mtrBR.getCurrentPosition()) / 2 < newRightTarget)) {
                double rem = (Math.abs(robot.driveTrain.mtrFL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrFR.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBR.getCurrentPosition())) / 4;
                double NLspeed;
                double NRspeed;
                //To Avoid spinning the wheels, this will "Slowly" ramp the motors up over
                //the amount of time you set for this SubRun
                double R = robot.driveTrain.runtime.seconds();
                if (R < rampup) {
                    double ramp = R / rampup;
                    NLspeed = Lspeed * ramp;
                    NRspeed = Rspeed * ramp;
                }
                //Keep running until you are about two rotations out
                else if (rem > (2000)) {
                    NLspeed = Lspeed;
                    NRspeed = Rspeed;
                }
                //start slowing down as you get close to the target
                else if (rem > (200) && (Lspeed * .2) > .1 && (Rspeed * .2) > .1) {
                    NLspeed = Lspeed * (rem / 1000);
                    NRspeed = Rspeed * (rem / 1000);
                }
                //minimum speed
                else {
                    NLspeed = Lspeed * .2;
                    NRspeed = Rspeed * .2;

                }
                //Pass the seed values to the motors
                robot.driveTrain.mtrFL.setPower(NLspeed);
                robot.driveTrain.mtrBL.setPower(NLspeed);
                robot.driveTrain.mtrFR.setPower(NRspeed);
                robot.driveTrain.mtrBR.setPower(NRspeed);
            }
            // Stop all motion;
            //Note: This is outside our while statement, this will only activate once the time, or distance has been met
            robot.driveTrain.mtrFL.setPower(0);
            robot.driveTrain.mtrFR.setPower(0);
            robot.driveTrain.mtrBL.setPower(0);
            robot.driveTrain.mtrBR.setPower(0);
            // show the driver how close they got to the last target
            telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d", robot.driveTrain.mtrFL.getCurrentPosition(), robot.driveTrain.mtrFR.getCurrentPosition());
            telemetry.update();
            //setting resetC as a way to check the current encoder values easily
            //double resetC = ((Math.abs(robot.driveTrain.mtrFL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBL.getCurrentPosition())+ Math.abs(robot.driveTrain.mtrFR.getCurrentPosition())+Math.abs(robot.driveTrain.mtrBR.getCurrentPosition())));
            //Get the motor encoder resets in motion
            //robot.driveTrain.mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.driveTrain.mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.driveTrain.mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.driveTrain.mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //keep waiting while the reset is running
            //while (Math.abs(resetC) > 0){
            //    resetC =  ((Math.abs(robot.driveTrain.mtrFL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBL.getCurrentPosition())+ Math.abs(robot.driveTrain.mtrFR.getCurrentPosition())+Math.abs(robot.driveTrain.mtrBR.getCurrentPosition())));
            //    idle();
            //}
            //sleep(1000);   // optional pause after each move
        }
    }
}