package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
import static android.os.SystemClock.sleep;

import static android.os.SystemClock.sleep;

@Autonomous(name = "inch test", group = "12596")

public class inchesTestAuto extends LinearOpMode {
    CopperHeadRobot robot = new CopperHeadRobot();

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    private int mineralAngle = 0; // angle to hit mineral
    private int wallAngle = -76; // angle for when we turn to depot from wall
    private static final double unlatchDist = -2;
    private static final double liftDist = 13;
    private double mineralDist = 22;
    private double backupDist = 8;
    private static final double markerDist = -50;
    private static final double toDepotDist = -32;
    private static final double depotToPark = 75;//incr by 10
    private static final double craterDist = 20;
    private int markerTurn = 80; // angle to trun to wall
    private int parkAngle = -2;
    private int easeIn = -15;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    boolean finished = false;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry, true);
        telemetry.addLine().addData("ARE THE PIZZA ROLLS READY?", robot.driveTrain.mtrBL.getCurrentPosition());
        while (!isStopRequested() && !robot.driveTrain.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        robot.liftAndHook.mtrLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftAndHook.mtrLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftAndHook.mtrLift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("imu calib status", robot.driveTrain.imu.getCalibrationStatus().toString());
        telemetry.update();
        boolean finished = false;
        boolean scanning = false;
        int position = 0;
        waitForStart();
        if (opModeIsActive() && finished == false) {
           // robot.collector.srvMarker.setPosition(0.95);
           //sleep(1000);
            robot.collector.srvMarker.setPosition(1);
            sleep(2000);
            robot.collector.srvMarker.setPosition(.15);
           finished = true;
        }

    }


    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    //private final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    //private final String LABEL_GOLD_MINERAL = "Gold Mineral";
    //private final String LABEL_SILVER_MINERAL = "Silver Mineral";
    //private final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
    //private VuforiaLocalizer vuforia;
    //private TFObjectDetector tfod;

    public void rotate(double degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        robot.driveTrain.resetAngle();
        telemetry.addLine().addData("Robot Angle", robot.driveTrain.getAngle());

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = -power;
            rightPower = power;
        } else if (degrees > 0) {   // turn left.
            leftPower = power;
            rightPower = -power;
        } else return;

        // set power to rotate.
        robot.driveTrain.mtrFL.setPower(leftPower);
        robot.driveTrain.mtrBL.setPower(leftPower);
        robot.driveTrain.mtrFR.setPower(rightPower);
        robot.driveTrain.mtrBR.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && robot.driveTrain.getAngle() == 0) {
            }

            while (opModeIsActive() && robot.driveTrain.getAngle() > degrees) {
            }
        } else {    // left turn.
            while (opModeIsActive() && robot.driveTrain.getAngle() < degrees) {
            }

            while (opModeIsActive() && robot.driveTrain.getAngle() > degrees) {
            }

            // turn the motors off.
            robot.driveTrain.stopMotors();

            // wait for rotation to stop.
            sleep(1000);

            // reset angle tracking on new heading.
            robot.driveTrain.resetAngle();
        }
    }

    private void unhangCrater() {
        robot.liftAndHook.mtrLift1.setPower(1);
        robot.liftAndHook.mtrLift2.setPower(1);
        sleep(1000);
        robot.liftAndHook.csrvPin.setPower(1);
        sleep(3000);
        robot.liftAndHook.csrvPin.setPower(0);
        robot.liftAndHook.stop();
        //rotate(-1, .25);
        robot.liftAndHook.goInches(-15, .4, 3);
        robot.liftAndHook.mtrLift1.setPower(-.25);
        robot.liftAndHook.mtrLift2.setPower(-.25);
        sleep(1000);
        robot.driveTrain.goInches(6, .2, 1);
        robot.liftAndHook.stop();
        robot.liftAndHook.goInches(10, .4, 3);
        robot.driveTrain.goInches(-6, .2, 1);
        rotate(-3, .4);
    }

    private void unhangDepot () {
        resetAngle();
        //double angle = getAngle();
        robot.liftAndHook.mtrLift1.setPower(1);
        robot.liftAndHook.mtrLift2.setPower(1);
        robot.liftAndHook.mtrLift3.setPower(1);
        sleep(250);
        robot.liftAndHook.csrvPin.setPower(1);
        sleep(1625);
        robot.liftAndHook.stop();
        //robot.liftAndHook.csrvPin.setPower(0);
        sleep(1000);
        robot.liftAndHook.csrvPin.setPower(0);
        robot.liftAndHook.goInches(-20, .4, 3);
        robot.liftAndHook.stop();
        rotate(-.64 * getAngle(), .4);
        sleep(125);
        robot.liftAndHook.timedRun();
        robot.liftAndHook.mtrLift1.setPower(-.4);
        robot.liftAndHook.mtrLift2.setPower(-.4);
        robot.liftAndHook.mtrLift3.setPower(-.4);
        sleep(125);
        robot.driveTrain.goInches(-2, .2, 1);
        robot.liftAndHook.stop();
        robot.liftAndHook.goInches(19, .6, 3);
        //robot.liftAndHook.goInches(-6, .4, 3);
        robot.driveTrain.goInches(2.5, .2, 1);
    }

    public void encoderDrive(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup) throws InterruptedException {

//        double COUNTS_PER_MOTOR_REV = 1120;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
//        double DRIVE_GEAR_REDUCTION = .5;     // This is the ratio between the motor axle and the wheel
//        double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        double COUNTS_PER_INCH = (robot.driveTrain.COUNTS_PER_MOTOR_REV * robot.driveTrain.DRIVE_GEAR_REDUCTION) /
                (robot.driveTrain.WHEEL_DIAMETER_INCHES * Math.PI);
        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        newLeftTarget = Math.abs(robot.driveTrain.mtrFL.getCurrentPosition() + robot.driveTrain.mtrBL.getCurrentPosition() / 2 + (int) (Inches * COUNTS_PER_INCH));
        newRightTarget = Math.abs(robot.driveTrain.mtrFR.getCurrentPosition() + robot.driveTrain.mtrBR.getCurrentPosition() / 2 + (int) (Inches * COUNTS_PER_INCH));
        // reset the timeout time and start motion.
        robot.driveTrain.runtime.reset();
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ((robot.driveTrain.runtime.seconds() < timeoutS) &&
                (Math.abs(robot.driveTrain.mtrFL.getCurrentPosition() + robot.driveTrain.mtrBL.getCurrentPosition()) / 2 < newLeftTarget &&
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
            else if (rem > (robot.driveTrain.COUNTS_PER_MOTOR_REV * 2)) {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if (rem > (200) && (Lspeed * .2) > .1 && (Rspeed * .2) > .1) {
                NLspeed = Lspeed * (rem / robot.driveTrain.COUNTS_PER_MOTOR_REV);
                NRspeed = Rspeed * (rem / robot.driveTrain.COUNTS_PER_MOTOR_REV);
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
        robot.driveTrain.mtrFL.setPower(0);
        robot.driveTrain.mtrFR.setPower(0);
        robot.driveTrain.mtrBL.setPower(0);
        robot.driveTrain.mtrBR.setPower(0);
        // show the driver how close they got to the last target
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
        telemetry.addData("Path2", "Running at %7d :%7d", robot.driveTrain.mtrFL.getCurrentPosition(), robot.driveTrain.mtrFR.getCurrentPosition());
        telemetry.update();
        //setting resetC as a way to check the current encoder values easily
//        double resetC = ((Math.abs(robot.driveTrain.mtrFL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrFR.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBR.getCurrentPosition())));
        //Get the motor encoder resets in motion
        robot.driveTrain.mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
//        while (Math.abs(resetC) > 0) {
//            resetC = ((Math.abs(robot.driveTrain.mtrFL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBR.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrFR.getCurrentPosition())));
//            idle();
//        }
        // switch the motors back to RUN_USING_ENCODER mode
        robot.driveTrain.mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveTrain.mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("test", "test");

        //give the encoders a chance to switch modes.
        sleep(500);
    }
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
    public void runToPositionDrive(double Lspeed, double Rspeed, double Inches, double timeoutS, double rampup) throws InterruptedException {

//        double COUNTS_PER_MOTOR_REV = 1120;    //Set for NevRest 20 drive. For 40's change to 1120. For 60's 1680
//        double DRIVE_GEAR_REDUCTION = .5;     // This is the ratio between the motor axle and the wheel
//        double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        double COUNTS_PER_INCH = (robot.driveTrain.COUNTS_PER_MOTOR_REV * robot.driveTrain.DRIVE_GEAR_REDUCTION) /
                (robot.driveTrain.WHEEL_DIAMETER_INCHES * Math.PI);
        //initialise some variables for the subroutine
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        robot.driveTrain.reset();
        // Determine new target position, and pass to motor controller we only do this in case the encoders are not totally zero'd
        newLeftTarget = robot.driveTrain.mtrFL.getCurrentPosition() + robot.driveTrain.mtrBL.getCurrentPosition() / 2 + (int) (Inches * COUNTS_PER_INCH);
        newRightTarget = robot.driveTrain.mtrFR.getCurrentPosition() + robot.driveTrain.mtrBR.getCurrentPosition() / 2 + (int) (Inches * COUNTS_PER_INCH);
        // reset the timeout time and start motion.
        robot.driveTrain.runtime.reset();
        robot.driveTrain.setMode();
        robot.driveTrain.rightAndLeftPosition(newLeftTarget, newRightTarget);
        // keep looping while we are still active, and there is time left, and neither set of motors have reached the target
        while ((robot.driveTrain.runtime.seconds() < timeoutS) &&
                (robot.driveTrain.mtrBR.isBusy() && robot.driveTrain.mtrBL.isBusy() && robot.driveTrain.mtrFR.isBusy() && robot.driveTrain.mtrFL.isBusy())) {
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
            else if (rem > (robot.driveTrain.COUNTS_PER_MOTOR_REV * 2)) {
                NLspeed = Lspeed;
                NRspeed = Rspeed;
            }
            //start slowing down as you get close to the target
            else if (rem > (200) && (Lspeed * .2) > .1 && (Rspeed * .2) > .1) {
                NLspeed = Lspeed * (rem / robot.driveTrain.COUNTS_PER_MOTOR_REV);
                NRspeed = Rspeed * (rem / robot.driveTrain.COUNTS_PER_MOTOR_REV);
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
        robot.driveTrain.mtrFL.setPower(0);
        robot.driveTrain.mtrFR.setPower(0);
        robot.driveTrain.mtrBL.setPower(0);
        robot.driveTrain.mtrBR.setPower(0);
        // show the driver how close they got to the last target
        telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
        telemetry.addData("Path2", "Running at %7d :%7d", robot.driveTrain.mtrFL.getCurrentPosition(), robot.driveTrain.mtrFR.getCurrentPosition());
        telemetry.update();
        //setting resetC as a way to check the current encoder values easily
//        double resetC = ((Math.abs(robot.driveTrain.mtrFL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrFR.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBR.getCurrentPosition())));
        //Get the motor encoder resets in motion
        //robot.driveTrain.mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.driveTrain.mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.driveTrain.mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.driveTrain.mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //keep waiting while the reset is running
//        while (Math.abs(resetC) > 0) {
//            resetC = ((Math.abs(robot.driveTrain.mtrFL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBL.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrBR.getCurrentPosition()) + Math.abs(robot.driveTrain.mtrFR.getCurrentPosition())));
//            idle();
//        }
        // switch the motors back to RUN_USING_ENCODER mode
        //robot.driveTrain.mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.driveTrain.mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.driveTrain.mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.driveTrain.mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //telemetry.addData("test", "test");

        //give the encoders a chance to switch modes.
        //sleep(500);
    }
    public void goCoast(double inches, double speed, double timeout) {
        robot.driveTrain.runtime.reset();
        robot.driveTrain.reset();
        robot.driveTrain.setMode();
        robot.driveTrain.targetPosition(inches);
        robot.driveTrain.move(speed);
        robot.driveTrain.timeoutExit(timeout);
        //robot.driveTrain.stopMotors();
        //robot.driveTrain.reset();
    }
    public void checkPosition() {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
            robot.collector.runtime.reset();
            while (finished == false && robot.collector.runtime.seconds() < 8) {
                if (tfod != null && finished == false && robot.collector.runtime.seconds() < 5) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int otherMineral = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    otherMineral = (int) recognition.getLeft();
                                }
                            }

                            if (goldMineralX  == -1) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                telemetry.addData("sadf",123);
                                mineralAngle = 54;
                                markerTurn = 87;
                                mineralDist = 26;
                                backupDist = 16;
                                parkAngle = 0;
                                wallAngle = -78;
                                finished = true;
                            } else if (goldMineralX < silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                telemetry.addData("sadf",123);
                                mineralAngle = 110;
                                mineralDist = 22;
                                backupDist = 12;
                                markerTurn = 36;
                                parkAngle = 0;
                                easeIn = -4;
                                finished = true;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                mineralAngle = 81;
                                markerTurn = 67;
                                backupDist = 10;
                                easeIn = -8;
                                finished = true;
                                telemetry.addData("sadf",123);
                            }
                        }
                        telemetry.update();
                    }
                }
                else {
                    telemetry.addData("Gold Mineral Position", "Center");
                    mineralAngle = 81;
                    markerTurn = 67;
                    backupDist = 10;
                    easeIn = -8;
                    finished = true;
                    telemetry.addData("sadf",123);

                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }
//    public void goTest(double inches, double speed, double timeout) {
//        robot.driveTrain.runtime.reset();
//        robot.driveTrain.reset();
//        robot.driveTrain.setMode();
//        robot.driveTrain.targetPosition(inches);
//        robot.driveTrain.move(speed);
//        robot.driveTrain.timeoutExit(timeout);
//        robot.driveTrain.stopMotors();
//        robot.driveTrain.reset();
//        telemetry.addData("mtrFL", robot.driveTrain.mtrFL.getCurrentPosition());
//        telemetry.addData("mtrBL", robot.driveTrain.mtrBL.getCurrentPosition());
//        telemetry.addData("mtrFR", robot.driveTrain.mtrFR.getCurrentPosition());
//        telemetry.addData("mtrBR", robot.driveTrain.mtrBR.getCurrentPosition());
//        telemetry.update();
//    }
//
//    public void timeoutExitTest(double seconds) {
//        robot.driveTrain.runtime.reset();
//        while (robot.driveTrain.runtime.seconds() < seconds && (robot.driveTrain.mtrBR.isBusy() && robot.driveTrain.mtrBL.isBusy() && robot.driveTrain.mtrFR.isBusy() && robot.driveTrain.mtrFL.isBusy())) {
//            telemetry.addData("Path1", "Running to target position");
//            telemetry.addData("mtrFL", robot.driveTrain.mtrFL.getCurrentPosition());
//            telemetry.addData("mtrBL", robot.driveTrain.mtrBL.getCurrentPosition());
//            telemetry.addData("mtrFR", robot.driveTrain.mtrFR.getCurrentPosition());
//            telemetry.addData("mtrBR", robot.driveTrain.mtrBR.getCurrentPosition());
//            telemetry.update();
//        }
//    }
}



