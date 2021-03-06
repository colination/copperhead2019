package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.List;

import static android.os.SystemClock.sleep;

public class DriveTrain extends robotPart {
    //motors
    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DcMotor mtrBL = null;
    public DcMotor mtrBR = null;

    DigitalChannel touch;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    public ElapsedTime runtime = new ElapsedTime();
    double COUNTS_PER_MOTOR_REV = 1120;
    double DRIVE_GEAR_REDUCTION = .5;
    double WHEEL_DIAMETER_INCHES = 4.0;
    double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);


    //sensors
    public void init(HardwareMap ahwmap, Telemetry myTelemetry, boolean enableIMU) {
        super.init(ahwmap, myTelemetry);
        // Motors
        mtrFL = ahwmap.dcMotor.get("mtrFL");
        mtrFR = ahwmap.dcMotor.get("mtrFR");
        mtrBL = ahwmap.dcMotor.get("mtrBL");
        mtrBR = ahwmap.dcMotor.get("mtrBR");
        // Set motor directions
        mtrFL.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrBL.setDirection(DcMotorSimple.Direction.FORWARD);
        mtrFR.setDirection(DcMotorSimple.Direction.REVERSE);
        mtrBR.setDirection(DcMotorSimple.Direction.REVERSE);
        // Set motors to not use encoders until specified
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //IMU
        imu = ahwmap.get(BNO055IMU.class, "imu");
        if (enableIMU) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;
            imu = ahwmap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
        }
        stopMotors();
    }

    public void stopMotors() {
        mtrFL.setPower(0);
        mtrBL.setPower(0);
        mtrFR.setPower(0);
        mtrBR.setPower(0);
    }

    public void setMode() {
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void reset() {
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void targetPosition(double inches) {
        mtrFL.setTargetPosition((int) (mtrFL.getCurrentPosition() - (inches * COUNTS_PER_INCH)));
        mtrFR.setTargetPosition((int) (mtrFR.getCurrentPosition() - (inches * COUNTS_PER_INCH)));
        mtrBL.setTargetPosition((int) (mtrBL.getCurrentPosition() - (inches * COUNTS_PER_INCH)));
        mtrBR.setTargetPosition((int) (mtrBR.getCurrentPosition() - (inches * COUNTS_PER_INCH)));
    }

    public void rightAndLeftPosition(double left, double right) {
        mtrFL.setTargetPosition((int) (mtrFL.getCurrentPosition() + (left)));
        mtrFR.setTargetPosition((int) (mtrFR.getCurrentPosition() + (right)));
        mtrBL.setTargetPosition((int) (mtrBL.getCurrentPosition() + (left)));
        mtrBR.setTargetPosition((int) (mtrBR.getCurrentPosition() + (right)));
    }


    public int target(double inches) {
        return (int) (inches * COUNTS_PER_INCH);
    }

    public void move(double power) {
        mtrFL.setPower(power);
        mtrFR.setPower(power);
        mtrBL.setPower(power);
        mtrBR.setPower(power);
    }

    public  void strafe(double power) {
        //right is positive, left is negative
        mtrFL.setPower(power);
        mtrBR.setPower(power);
        mtrFR.setPower(-power);
        mtrBL.setPower(-power);
    }

    public void moveLean(double power, double shift) {
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

    public void timeoutExit(double seconds) {
        runtime.reset();
        while (runtime.seconds() < seconds && (mtrBR.isBusy() && mtrBL.isBusy() && mtrFR.isBusy() && mtrFL.isBusy())) {
            privateTelemetry.addData("Path1", "Running to target position");
            privateTelemetry.addData("Path2", "Running at:",
                    mtrBL.getCurrentPosition(),
                    mtrBR.getCurrentPosition(),
                    mtrFL.getCurrentPosition(),
                    mtrFR.getCurrentPosition());
            privateTelemetry.update();
        }
    }

    public void goInches(double inches, double speed, double timeout) {
        runtime.reset();
        reset();
        setMode();
        targetPosition(inches);
        move(speed);
        timeoutExit(timeout);
        stopMotors();
        reset();

    }
    public void gyroInches(double inches, double leftSpeed, double rightSpeed,  double timeout) {
        runtime.reset();
        reset();
        setMode();
        targetPosition(inches);
        Tank(leftSpeed, rightSpeed);
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

    public double encoderAvg() {
        return (mtrFR.getCurrentPosition() + mtrFL.getCurrentPosition() + mtrBR.getCurrentPosition() + mtrBL.getCurrentPosition()) / 4;
    }

    public void PInches(double inches, double power, double rampTime) {
        reset();
        setMode();
        targetPosition(inches);
        int distance = target(inches);
        double rampSpeed = 0;
        runtime.reset();
        while (runtime.seconds() < rampTime) {
            rampSpeed = 0.2 * (runtime.seconds()/rampTime);
        }
        while (mtrBL.isBusy() && mtrBR.isBusy() && mtrFL.isBusy() && mtrFR.isBusy()) {
            move(rampSpeed + ((power - .2) * (Math.abs(distance) - Math.abs(encoderAvg()) / Math.abs(distance))));
        }
        stopMotors();
    }

    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle() {
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

    public double checkDirection() {
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void rotate(double degrees, double power)
    {
        turnMode();
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();
        privateTelemetry.addLine().addData("Robot Angle", getAngle());

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
        mtrFL.setPower(leftPower);
        mtrBL.setPower(leftPower);
        mtrFR.setPower(rightPower);
        mtrBR.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {
//                mtrFL.setPower(leftPower);
//                mtrBL.setPower(leftPower);
//                mtrFR.setPower(rightPower);
//                mtrBR.setPower(rightPower);
                privateTelemetry.addData("degrees", getAngle());
                privateTelemetry.update();
            }
        }
        else    // left turn.
            while (getAngle() < degrees) {
//                mtrFL.setPower(leftPower);
//                mtrBL.setPower(leftPower);
//                mtrFR.setPower(rightPower);
//                mtrBR.setPower(rightPower);
                privateTelemetry.addData("degrees", getAngle());
                privateTelemetry.update();
            }

        // turn the motors off.
        stopMotors();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public double pidDrive(double kP,double kI, double kD, double target, DcMotor motor){
        double proportional, integral = 0, derivative, error, power = 0;
        runtime.reset();
        double time = runtime.milliseconds();
        double lastTime = 0;
        double lastError = 0;
        error = target - motor.getCurrentPosition();
        while(Math.abs(error) > 5 && privateOpMode.opModeIsActive()){
            proportional = error * kP;
            integral += error * (runtime.milliseconds() - lastTime) * kI;
            derivative =  (error - lastError) / (runtime.milliseconds() - lastTime) * kD;
            power = proportional + integral + derivative;
            //motor.setPower(power);
            lastTime = runtime.milliseconds();
            lastError = error;
        }
        return power;
    }
}




