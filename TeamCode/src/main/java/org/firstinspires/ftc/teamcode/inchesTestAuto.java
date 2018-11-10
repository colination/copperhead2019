package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

import static android.os.SystemClock.sleep;

@Autonomous(name = "Inch Test", group = "12596")
public class inchesTestAuto extends LinearOpMode {
    CopperHeadRobot robot = new CopperHeadRobot();



    //private TestCV detector;
    //detector = new TestCV();
    //detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
    //detector.useDefaults();
    //detector.downscale = 0.4; // How much to downscale the input frames
    //detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
    //detector.maxAreaScorer.weight = 0.005;
    //detector.ratioScorer.weight = 5;
    //detector.ratioScorer.perfectRatio = 1.0;
    //detector.enable();


    //private TestCV detector;

        //detector = new TestCV();
        //detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        //detector.useDefaults();
        //detector.downscale = 0.4; // How much to downscale the input frames
        //detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.maxAreaScorer.weight = 0.005;
        //detector.ratioScorer.weight = 5;
        //detector.ratioScorer.perfectRatio = 1.0;
        //detector.enable();

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    //  List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
    BNO055IMU               imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        while (!isStopRequested() && !robot.driveTrain.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        robot.liftAndHook.mtrLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftAndHook.mtrLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("imu calib status", robot.driveTrain.imu.getCalibrationStatus().toString());

        telemetry.update();
        boolean finished = false;
        boolean scanning = false;
        int position = 0;
        waitForStart();
        if (opModeIsActive() && finished == false) {

//            if (tfod != null) {
//                tfod.activate();
//            }
//            if (tfod != null && scanning == false) {
//                // getUpdatedRecognitions() will return null if no new information is available since
//                // the last time that call was made.
//                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                if (updatedRecognitions != null) {
//                    telemetry.addData("# Object Detected", updatedRecognitions.size());
//                    if (updatedRecognitions.size() == 3) {
//                        int goldMineralX = -1;
//                        int silverMineral1X = -1;
//                        int silverMineral2X = -1;
//                        for (Recognition recognition : updatedRecognitions) {
//                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                                goldMineralX = (int) recognition.getLeft();
//                            } else if (silverMineral1X == -1) {
//                                silverMineral1X = (int) recognition.getLeft();
//                            } else {
//                                silverMineral2X = (int) recognition.getLeft();
//                            }
//                        }
//                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                                telemetry.addData("Gold Mineral Position", "Left");
//                                position = 1;
//                                scanning = true;
//                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                telemetry.addData("Gold Mineral Position", "Right");
//                                position = 3;
//                                scanning = true;
//                            } else {
//                                telemetry.addData("Gold Mineral Position", "Center");
//                                position = 2;
//                                scanning = true;
//                            }
//                        }
//                    }
//                    telemetry.update();
//                }
//            }
//            sleep(2000);
//            telemetry.addData("Position", position);
//            telemetry.update();
//            robot.driveTrain.rotate(turn,.3);

            robot.liftAndHook.goInches(12, .8, 6);
            robot.driveTrain.goInches(-4, .2, 4);
            robot.driveTrain.setSideRoller(1);
            robot.liftAndHook.goInches(-12, .8, 6);
            //robot.driveTrain.rotate(45, .25);
            //robot.driveTrain.goInches(32,.5,12);
            //robot.liftAndHook.goInches(12, .8, 6);
           // robot.driveTrain.goInches(-4, .2, 4);
           // robot.driveTrain.setSideRoller(1);
            //robot.liftAndHook.goInches(-12, .8, 6);
            //robot.driveTrain.rotate(90, .25);
            //robot.driveTrain.stopMotors();

//            robot.liftAndHook.goInches(-11.5, .8, 4); // move up to lower down to ground
//            robot.driveTrain.goInches(-.75, .2, 2); // move off latch
//            robot.driveTrain.setSideRoller(.4); // move the side roller down
//            robot.liftAndHook.goInches( 11.5, .8, 4); // move the lift back down

            // robot.driveTrain.rotate(-120, .25); // rotate towards right mineral
            //Left Position
//            if (position == 1){
//                robot.driveTrain.rotate(-58,.25);
//                robot.driveTrain.goInches(26,.5,12);
//            }
//
//            //Center
//            if (position == 2){
//                robot.driveTrain.rotate(-88,.25);
//                robot.driveTrain.goInches(21,.5,12);}
//
//            //Right
//            if (position == 3){
//                robot.driveTrain.rotate(-120,.25);
//                robot.driveTrain.goInches(23,.5,12);}

            // robot.driveTrain.rotate(-58, .25); // (??) rotate towards left mineral

//            sleep(250);
//            robot.driveTrain.goInches(25, .4, 6); // run into the assigned mineral and park
//            robot.driveTrain.stopMotors(); // stop the motors
            finished = true;
        }
//        if (tfod != null) {
//            tfod.shutdown();
//        }
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
//    public void rotate(int degrees, double power) {
//        double leftPower, rightPower;
//
//        /*public void rotate ( int degrees, double power) {
//            double leftPower, rightPower;
//>>>>>>> d0aad962f1dec2721eb3a0526a1fb1cde61029a2
//
//            // restart imu movement tracking.
//            robot.driveTrain.resetAngle();
//            telemetry.addLine().addData("Robot Angle", robot.driveTrain.getAngle());
//
//            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//            // clockwise (right).
//
//            if (degrees < 0) {   // turn right.
//                leftPower = -power;
//                rightPower = power;
//            }
//            else if (degrees > 0) {   // turn left.
//                leftPower = power;
//                rightPower = -power;
//            }
//            else return;
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
//                while (opModeIsActive() && robot.driveTrain.getAngle() == 0) {}
//
//<<<<<<< HEAD
//                while (opModeIsActive() && robot.driveTrain.getAngle() > degrees) {}
//            }
//            else    // left turn.
//                while (opModeIsActive() && robot.driveTrain.getAngle() < degrees) {}
//=======
//                while (opModeIsActive() && robot.driveTrain.getAngle() > degrees) {
//                }
//            } else    // left turn.
//                while (opModeIsActive() && robot.driveTrain.getAngle() < degrees) {
//                }
//>>>>>>> 832887b6f747ca24ab7d74bf80bfbbd26583e306
//
//            // turn the motors off.
//            robot.driveTrain.stopMotors();
//>>>>>>> 91b16c4f952570441ac88f78c7a535a06446e293
//
//
//<<<<<<< HEAD
//
//=======
//            // reset angle tracking on new heading.
//            robot.driveTrain.resetAngle();
//        }
//<<<<<<< HEAD
//}*/
//
//
//        // reset angle tracking on new heading.
//        robot.driveTrain.resetAngle();
//    }
}