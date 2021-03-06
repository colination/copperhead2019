// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses IMU to drive in a straight line when not avoiding an obstacle.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="crater auto", group="12596")

public class Autotestingangles extends LinearOpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private int mineralAngle = 0;
    private int wallAngle = 61;
    private static final double unlatchDist = -2;
    private static final double liftDist = 23;
    private double mineralDist = 18;
    private double backupDist = 8;
    private static final double markerDist = -40;
    private static double toDepotDist = -34;
    private static final double depotToPark = 61;
    private static final double craterDist = 20;
    private int markerTurn = 80;
    private int parkAngle = -2;
    private int wallEaseIn = -15;



    private int mineralDistance;
    boolean finished = false;

    DigitalChannel          touch;
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;


     // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry, true);
        telemetry.addLine().addData("unhooks from lander, turns 90 degrees, and runs into a mineral/crater",robot.driveTrain.mtrBL.getCurrentPosition());

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

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        /*if (tfod != null) {
            tfod.activate();
        }
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        telemetry.addData("# Object Detected", updatedRecognitions.size());
        telemetry.update();*/


        /** Wait for the game to begin */

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.addData("abc","123");
        telemetry.update();
        waitForStart();
        //test
        if (opModeIsActive()) {
            //robot.collector.srvMarker.setPosition(0.95);
            checkPosition(); // scan minerals and determine position of mineral
            while (mineralAngle == 0) {
                idle();
            }
            // Unhook
            unhangCrater();
            telemetry.addLine().addData("turning", getAngle());
            rotate(mineralAngle, .3); // rotate towards mineral
            telemetry.addLine().addData("turnt", getAngle());
            sleep(250);
            // Run into mineral
            robot.driveTrain.goInches(-mineralDist, .3, 2);
            sleep(200);
            // Back up from mineral
            robot.driveTrain.goInches(backupDist,.3,2);
            // Turn towards wall
            rotate(markerTurn,.3);
            // drive to wall
            robot.driveTrain.goInches(toDepotDist, .3,3);
            robot.driveTrain.goInches(wallEaseIn, .15, 3);
            // Back up slightly from wall
            robot.driveTrain.goInches(3,.15,4);
            // turn towards depot
            rotate(wallAngle,.3);
            //drive into depot
            robot.driveTrain.goInches(markerDist, .7,3);
            //drop marker
            robot.collector.srvMarker.setPosition(0);
            // angle inwards towards wall
            rotate(7, .4);
            // drive to park in crater
            robot.driveTrain.goInches(depotToPark,.7,5);
            // slight adjustment to ensure a park
            rotate(-2, .4);
            robot.driveTrain.move(-.2);
            sleep(25000); // end sleep
        }
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
    public void rotate(double degrees, double power)
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
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    //private static final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    //private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    //private TFObjectDetector tfod;


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
                                mineralAngle = 57;
                                markerTurn = 88;
                                mineralDist = 26;
                                backupDist = 14.5;
                                parkAngle = 0;
                                toDepotDist -= 8;
                                finished = true;
                            } else if (goldMineralX < silverMineral1X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                telemetry.addData("sadf",123);
                                mineralAngle = 109;
                                mineralDist = 25;
                                backupDist = 11;
                                markerTurn = 35;
                                parkAngle = 0;
                                wallEaseIn = 0;
                                finished = true;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                mineralAngle = 85;
                                mineralDist = 22;
                                markerTurn = 70;
                                wallEaseIn = -13;
                                wallAngle = 60;
                                backupDist = 10;
                                finished = true;
                                telemetry.addData("sadf",123);
                            }
                        }
                        telemetry.update();
                    }
                }
                else {
                    mineralAngle = 85;
                    markerTurn = 68;
                    wallEaseIn = -10;
                    finished = true;
                    telemetry.addData("sadf",123);

                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters vParameters = new VuforiaLocalizer.Parameters();

        vParameters.vuforiaLicenseKey = VUFORIA_KEY;
        vParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(vParameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
    private void unhangCrater () {
        resetAngle();
        //double angle = getAngle();
        robot.liftAndHook.mtrLift1.setPower(1);
        robot.liftAndHook.mtrLift2.setPower(1);
        robot.liftAndHook.mtrLift3.setPower(1);
        sleep(250);
        robot.liftAndHook.csrvPin.setPower(1);
        sleep(1625);
        robot.liftAndHook.stop();
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
        robot.driveTrain.goInches(2, .2, 1);
        robot.liftAndHook.stop();
        robot.liftAndHook.goInches(19, .6, 3);
        //robot.liftAndHook.goInches(-6, .4, 3);
        robot.driveTrain.goInches(-2.5, .2, 1);
    }
    public void goInches(double inches, double speed, double timeout) {
        robot.driveTrain.runtime.reset();
        robot.driveTrain.reset();
        robot.driveTrain.setMode();
        robot.driveTrain.targetPosition(inches);
        robot.driveTrain.move(speed);
        robot.driveTrain.timeoutExit(timeout);
        robot.driveTrain.stopMotors();
        idle();
        //robot.driveTrain.reset();
    }
}