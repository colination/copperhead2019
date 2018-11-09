package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;

import static android.os.SystemClock.sleep;

@Autonomous(name = "Inch Test!23", group = "12596")
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

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    //  List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        while (!isStopRequested() && !robot.driveTrain.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }
        robot.liftAndHook.mtrLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftAndHook.mtrLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("imu calib status", robot.driveTrain.imu.getCalibrationStatus().toString());

        telemetry.update();
        boolean finished = false;
        boolean scanning = false;
        int turn = 0;
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
//                                scanning = true;
//                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                                telemetry.addData("Gold Mineral Position", "Right");
//                                turn = 90;
//                                scanning = true;
//                            } else {
//                                telemetry.addData("Gold Mineral Position", "Center");
//                                scanning = true;
//                            }
//                        }
//                    }
//                    telemetry.update();
//                }
//            }
//            robot.driveTrain.rotate(turn,.3);

            robot.liftAndHook.goInches(12, .8, 6);
            robot.driveTrain.goInches(-4, .2, 4);
//            robot.driveTrain.setSideRoller(1);
            robot.liftAndHook.goInches(-12, .8, 6); // <<< AUTO FOR CRATER UNHOOK NO VUFORIA
            robot.driveTrain.rotate(45, .25);
            robot.driveTrain.goInches(32,.5,12);
            robot.driveTrain.stopMotors();
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
}
