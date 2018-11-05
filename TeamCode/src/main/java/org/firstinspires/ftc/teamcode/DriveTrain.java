package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class DriveTrain extends robotPart {
    //motors
    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DcMotor mtrBL = null;
    public DcMotor mtrBR = null;
    DigitalChannel touch;
    BNO055IMU               imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;


    public Servo sideRoller = null;

    //servos
    public Servo srvRoller = null;

    public ElapsedTime runtime = new ElapsedTime();
    double     COUNTS_PER_MOTOR_REV    = 1120 ;
    double     DRIVE_GEAR_REDUCTION    = 0.5 ;
    double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);


    //sensors


    public void init(HardwareMap ahwmap, Telemetry myTelemetry){
        super.init(ahwmap, myTelemetry);
        //Motors
        mtrFL = ahwmap.dcMotor.get("mtrFL");
        mtrFR = ahwmap.dcMotor.get("mtrFR");
        mtrBL = ahwmap.dcMotor.get("mtrBL");
        mtrBR = ahwmap.dcMotor.get("mtrBR");

        mtrFL.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrBL.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrFR.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrBR.setDirection(DcMotorSimple.Direction.REVERSE);


        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Servos
        srvRoller = ahwmap.servo.get("srvRoller");
        srvRoller.setPosition(0);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        
        //mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopMotors();
    }

    public void stopMotors(){
        mtrFL.setPower(0);
        mtrBL.setPower(0);
        mtrFR.setPower(0);
        mtrBR.setPower(0);
    }
    public void setMode(){
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void reset() {
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void targetPosition(double inches){
        mtrFL.setTargetPosition((int) (mtrFL.getCurrentPosition() + (-inches * COUNTS_PER_INCH)));
        mtrFR.setTargetPosition((int) (mtrFR.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        mtrBL.setTargetPosition((int) (mtrBL.getCurrentPosition() + (-inches * COUNTS_PER_INCH)));
        mtrBR.setTargetPosition((int) (mtrBR.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
    }
    public void target(double inches) {
        mtrFL.setTargetPosition((int) (mtrFL.getCurrentPosition() + (-inches * COUNTS_PER_INCH)));
        mtrFR.setTargetPosition((int) (mtrFR.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        mtrBL.setTargetPosition((int) (mtrBL.getCurrentPosition() + (-inches * COUNTS_PER_INCH)));
        mtrBR.setTargetPosition((int) (mtrBR.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
    }

    public void move(double power){
        mtrFL.setPower(power);
        mtrFR.setPower(power);
        mtrBL.setPower(power);
        mtrBR.setPower(power);
    }

    public void moveLean(double power, double shift){
        mtrFL.setPower(power + shift);
        mtrBL.setPower(power + shift);
        mtrFR.setPower(power - shift);
        mtrBR.setPower(power - shift);
    }

    public void Tank(double leftPower, double rightPower) {
        mtrFL.setPower(leftPower);
        mtrBL.setPower(leftPower);
        mtrFR.setPower(rightPower);
        mtrBR.setPower(rightPower);
    }


    public void turnMode() {
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void timeoutExit(double seconds){
        runtime.reset();
        while (runtime.seconds() < seconds || (mtrBR.isBusy() && mtrBL.isBusy() && mtrFR.isBusy() && mtrFL.isBusy())){
            privateTelemetry.addData("Path1", "Running to target position");
            privateTelemetry.addData("Path2","Running at:",
                    mtrBL.getCurrentPosition(),
                    mtrBR.getCurrentPosition(),
                    mtrFL.getCurrentPosition(),
                    mtrFR.getCurrentPosition());
                    privateTelemetry.update();
        }
    }

    public void goInches(double inches, double speed, double timeout){
        runtime.reset();
        reset();
        setMode();
        targetPosition(inches);
        move(speed);
        timeoutExit(timeout);
        stopMotors();
        reset();
    }

    public void goLean(double inches, double power, double timeout, boolean direction) {
        double powerShift;
        //true for direction is forward, false for direction is backwards
        if (direction == true) {
            powerShift = .15;
        } else {
            powerShift = -.15;
        }
        runtime.reset();
        reset();
        setMode();
        targetPosition(inches);
        moveLean(power, powerShift);
        timeoutExit(timeout);
        stopMotors();
        reset();
    }

    public void setSideRoller(double Position) {
        srvRoller.setPosition(Position);

        }
    public void gyroInches(double inches) {
        reset();
        setMode();
        targetPosition(inches);

    }

    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();
        telemetry.addLine().addData("Robot Angle", getAngle());

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
        mtrFL.setPower(leftPower);
        mtrBL.setPower(leftPower);
        mtrFR.setPower(rightPower);
        mtrBR.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        stopMotors();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}
