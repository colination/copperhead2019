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

    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;
    //  List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap,telemetry);
        robot.liftAndHook.mtrLiftL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftAndHook.mtrLiftR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("ticks", robot.driveTrain.mtrBL.getCurrentPosition());
        telemetry.update();
        boolean finished = false;
        waitForStart();
        if (opModeIsActive() && finished == false){
            robot.liftAndHook.goInches(12,.8,6);

            robot.driveTrain.goInches(-4,.2,4);
            robot.driveTrain.setSideRoller(1);
            robot.liftAndHook.goInches(-12, .8, 6); // <<< AUTO FOR CRATER UNHOOK NO VUFORIA
            // rotate(90, .25);
            // robot.driveTrain.goInches(20,.5,12);
            robot.driveTrain.stopMotors();
            robot.driveTrain.goInches(-8,.5,4);
            //robot.driveTrain.setSideRoller(1);
            robot.liftAndHook.goInches(-12, .8, 6);
//            rotate(90, .25);
//            robot.driveTrain.goInches(20,.5,12);
//            robot.driveTrain.stopMotors();
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

    public void rotate(int degrees, double power)
    {
        //private final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
        //private final String LABEL_GOLD_MINERAL = "Gold Mineral";
        //private final String LABEL_SILVER_MINERAL = "Silver Mineral";
        //private final String VUFORIA_KEY = "AYW0N2f/////AAABmT84r6SmN0sChsfyQEho5YdE8Og8poAwDZNV1kfc3qS0dk+45j/4jRV4/nQRE5A8/X4+dSgUpEZWiRaCemAh3sc5xw7EM8FH0kJlk8ExI2q6pg14KXs90dNDyDQWSm7V2WzkC/gIfRAICgCs5CmOE4P/iZ51zzQaYyYT+lGay0QFFhVhYjRaSdWPmijDWGqg3q+S6FIanvM2yHVbiKdOmHpV5aml1KjRgMzG258F9R1vThPPe6OY8O0TwTAK2FF514CX8zJUbS5gbjcoA6VDrCoaYZoxfJylyikeSYlGWXnSlOJqoj3PxxDiZRvMwseAnqcJ2nNwIDccYQRk5Er3rTv4lYNLuRgqbyepot2NNN7d";
        //private VuforiaLocalizer vuforia;
        //private TFObjectDetector tfod;
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
            } else    // left turn.
                while (opModeIsActive() && robot.driveTrain.getAngle() < degrees) {
                }

            // turn the motors off.
            robot.driveTrain.stopMotors();

            // wait for rotation to stop.
            sleep(1000);

            // reset angle tracking on new heading.
            robot.driveTrain.resetAngle();
        }
}

        // reset angle tracking on new heading.





