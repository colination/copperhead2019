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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="Gyro Test Auto", group="12596")

public class gyroTestAuto extends LinearOpMode {
    CopperHeadRobot robot = new CopperHeadRobot();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private int mineralAngle = 0;
    private static final double unlatchDist = -1;
    private static final double liftDist = 14.5;
    private static final double mineralDist = 20;
    private static final double backupDist = -4 ;
    private static final double markerDist = -42;
    private static final double depotDist = 40;
    private static final double depotToPark = 65;
    private static final double craterDist = 20;
    private static final int markerTurn = 40;

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
        telemetry.addLine().addData("unhooks from lander, turns 90 degrees, and runs into a mineral/crater", robot.driveTrain.mtrBL.getCurrentPosition());

        // get a reference to REV Touch sensor.
        //touch = hardwareMap.digitalChannel.get("touch_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
//        initVuforia();

//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            initTfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
//        }

        /** Wait for the game to begin */

        telemetry.addData(">", "Press Play to start tracking");
        telemetry.addData("abc", "123");
        telemetry.update();
        waitForStart();
        //test
        if (opModeIsActive()) {
//            // Check position of gold, and set turn angle
//            checkPosition();
//            while (mineralAngle == 0) {
//                idle();
//            }
//            // Unhook
//            robot.liftAndHook.goInches(-liftDist, .5, 3); // move up to lower down to ground
//            robot.driveTrain.goInches(unlatchDist, .2, 1); // move off latch
//            robot.liftAndHook.goInches(liftDist, .5, 3);// move the lift back down
//            telemetry.addLine().addData("turning", getAngle());
//            rotate(mineralAngle, .4); // rotate towards the correct mineral
//            robot.driveTrain.goInches(mineralDist, .4, 2); // move forward and hit the correct mineral
//            // Path for marker
//            robot.driveTrain.goInches(backupDist,.4,2); // back up away from the mineral
//            rotate(-85,.4); // rotate to line up to the wall
//            robot.driveTrain.goInches(markerDist,.25,7); // move to hit the wall
//            robot.driveTrain.goInches(2,.25,7); // back up from wall slightly
//            rotate(33, .4); // rotate to line up with wall
//            robot.driveTrain.goInches(-depotDist, .25, 7); // move into the depot
//            rotate(7, .4); // rotate to line up with the wall again
//            robot.driveTrain.goInches(depotToPark,.7,5); // move back to park
//            while (opModeIsActive()) {
//                robot.driveTrain.move(.2); // adds a little bit more power to keep the robot from sliding off the crater
//            }
//        }
//        robot.driveTrain.stopMotors(); // turen the motors off
            robot.liftAndHook.angleTurned(robot.liftAndHook.potentiometer);
        }

        /**
         * Resets the cumulative angle tracking to zero.
         */
//        private void resetAngle ()
//        {
//            lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            globalAngle = 0;
//        }
//
//        /**
//         * Get current cumulative angle rotation from last reset.
//         * @return Angle in degrees. + = left, - = right.
//         */
//        private double getAngle ()
//        {
//            // We experimentally determined the Z axis is the axis we want to use for heading angle.
//            // We have to process the angle because the imu works in euler angles so the Z axis is
//            // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
//            // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
//
//            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//            if (deltaAngle < -180)
//                deltaAngle += 360;
//            else if (deltaAngle > 180)
//                deltaAngle -= 360;
//
//            globalAngle += deltaAngle;
//
//            lastAngles = angles;
//
//            return globalAngle;
//        }
//
//        /**
//         * See if we are moving in a straight line and if not return a power correction value.
//         * @return Power adjustment, + is adjust left - is adjust right.
//         */
//        private double checkDirection ()
//        {
//            // The gain value determines how sensitive the correction is to direction changes.
//            // You will have to experiment with your robot to get small smooth direction changes
//            // to stay on a straight line.
//            double correction, angle, gain = .10;
//
//            angle = getAngle();
//
//            if (angle == 0)
//                correction = 0;             // no adjustment.
//            else
//                correction = -angle;        // reverse sign of angle for correction.
//
//            correction = correction * gain;
//
//            return correction;
//        }

        /**
         * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
         * @param degrees Degrees to turn, + is left - is right
         */
//        public void rotate ( int degrees, double power)
//        {
//            robot.driveTrain.turnMode();
//            double leftPower, rightPower;
//
//            // restart imu movement tracking.
//            resetAngle();
//            telemetry.addLine().addData("Robot Angle", getAngle());
//
//            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//            // clockwise (right).
//
//            if (degrees > 0) {   // turn right.
//                leftPower = power;
//                rightPower = -power;
//            } else if (degrees < 0) {   // turn left.
//                leftPower = -power;
//                rightPower = power;
//            } else return;
//
//            // set power to rotate.
//            robot.driveTrain.mtrFL.setPower(leftPower);
//            robot.driveTrain.mtrBL.setPower(leftPower);
//            robot.driveTrain.mtrFR.setPower(rightPower);
//            robot.driveTrain.mtrBR.setPower(rightPower);
//
//            // rotate until turn is completed.
//            if (degrees < 0) {
//                // On right turn we have to get off zero first.
//                while (opModeIsActive() && getAngle() == 0) {
//                }
//
//                while (opModeIsActive() && getAngle() > degrees) {
//                    robot.driveTrain.mtrFL.setPower(leftPower);
//                    robot.driveTrain.mtrBL.setPower(leftPower);
//                    robot.driveTrain.mtrFR.setPower(rightPower);
//                    robot.driveTrain.mtrBR.setPower(rightPower);
//                    telemetry.addData("degrees", getAngle());
//                    telemetry.update();
//                }
//            } else    // left turn.
//                while (opModeIsActive() && getAngle() < degrees) {
//                    robot.driveTrain.mtrFL.setPower(leftPower);
//                    robot.driveTrain.mtrBL.setPower(leftPower);
//                    robot.driveTrain.mtrFR.setPower(rightPower);
//                    robot.driveTrain.mtrBR.setPower(rightPower);
//                    telemetry.addData("degrees", getAngle());
//                    telemetry.update();
//                }
//
//            // turn the motors off.
//            robot.driveTrain.stopMotors();
//
//            // wait for rotation to stop.
//            sleep(1000);
//
//            // reset angle tracking on new heading.
//            resetAngle();
//        }
//        /*
//         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
//         * web site at https://developer.vuforia.com/license-manager.
//         *
//         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
//         * random data. As an example, here is a example of a fragment of a valid key:
//         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
//         * Once you've obtained a license key, copy the string from the Vuforia web site
//         * and paste it in to your code on the next line, between the double quotes.
//         */
//        //private static final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
//
//        /**
//         * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//         * localization engine.
//         */
//        //private VuforiaLocalizer vuforia;
//
//        /**
//         * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
//         * Detection engine.
//         */
//        //private TFObjectDetector tfod;
//
//
//        public void checkPosition () {
//            if (opModeIsActive()) {
//                /** Activate Tensor Flow Object Detection. */
//                if (tfod != null) {
//                    tfod.activate();
//                }
//                while (finished == false) {
//                    if (tfod != null && finished == false) {
//                        // getUpdatedRecognitions() will return null if no new information is available since
//                        // the last time that call was made.
//                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                        if (updatedRecognitions != null) {
//                            telemetry.addData("# Object Detected", updatedRecognitions.size());
//                            if (updatedRecognitions.size() == 2) {
//                                int goldMineralX = -1;
//                                int silverMineral1X = -1;
//                                int otherMineral = -1;
//                                for (Recognition recognition : updatedRecognitions) {
//                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                                        goldMineralX = (int) recognition.getLeft();
//                                    } else if (silverMineral1X == -1) {
//                                        silverMineral1X = (int) recognition.getLeft();
//                                    } else {
//                                        otherMineral = (int) recognition.getLeft();
//                                    }
//                                }
//
//                                if (goldMineralX == -1) {
//                                    telemetry.addData("Gold Mineral Position", "Right");
//                                    telemetry.addData("sadf", 123);
//                                    mineralAngle = -102;
//                                    finished = true;
//                                } else if (goldMineralX < silverMineral1X) {
//                                    telemetry.addData("Gold Mineral Position", "Left");
//                                    telemetry.addData("sadf", 123);
//                                    mineralAngle = -45;
//                                    finished = true;
//                                } else {
//                                    telemetry.addData("Gold Mineral Position", "Center");
//                                    mineralAngle = -70;
//                                    finished = true;
//                                    telemetry.addData("sadf", 123);
//                                }
//                            }
//                            telemetry.update();
//                        }
//                    }
//                }
//            }
//            if (tfod != null) {
//                tfod.shutdown();
//            }
//        }
//
//        /**
//         * Initialize the Vuforia localization engine.
//         */
//        private void initVuforia () {
//            /*
//             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//             */
//            VuforiaLocalizer.Parameters vParameters = new VuforiaLocalizer.Parameters();
//
//            vParameters.vuforiaLicenseKey = VUFORIA_KEY;
//            vParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//            //  Instantiate the Vuforia engine
//            vuforia = ClassFactory.getInstance().createVuforia(vParameters);
//
//            // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
//        }
//
//        /**
//         * Initialize the Tensor Flow Object Detection engine.
//         */
//        private void initTfod () {
//            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
//        }

    }
}