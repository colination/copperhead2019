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

import static android.os.SystemClock.sleep;

public class DriveTrain extends robotPart {
    //motors
    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DcMotor mtrBL = null;
    public DcMotor mtrBR = null;
    public Servo sideRoller = null;

    DigitalChannel touch;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    //servos
    public Servo srvRoller = null;

    //BNO055IMU               imu;
    //Orientation             lastAngles = new Orientation();
    //double globalAngle, power = .30, correction;

    public ElapsedTime runtime = new ElapsedTime();
    double COUNTS_PER_MOTOR_REV = 1120;
    double DRIVE_GEAR_REDUCTION = .5;
    double WHEEL_DIAMETER_INCHES = 4.0;
    double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);


    //sensors


    public void init(HardwareMap ahwmap, Telemetry myTelemetry) {
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


        //Imu

        //Servos
        srvRoller = ahwmap.servo.get("srvRoller");
<<<<<<< HEAD

        srvRoller.setPosition(0.4);

        srvRoller.setPosition(0.4);

        srvRoller.setPosition(0);

        srvRoller.setPosition(0.4);

=======
>>>>>>> 832887b6f747ca24ab7d74bf80bfbbd26583e306
        srvRoller.setPosition(0.4);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;


        //Servos
        srvRoller = ahwmap.servo.get("srvRoller");
        //srvRoller.setPosition(-1);
        //mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        mtrFL.setTargetPosition((int) (mtrFL.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        mtrFR.setTargetPosition((int) (mtrFR.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        mtrBL.setTargetPosition((int) (mtrBL.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        mtrBR.setTargetPosition((int) (mtrBR.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
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

    public void goLean(double inches, double power, double timeout, boolean direction) {
        double powerShift;
        //true for direction is forward, false for direction is backwards


        if (direction == true) {
            powerShift = .1;
        } else {
            powerShift = -.1;

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

            if (direction == true) {
                powerShift = .15;
            } else {
                powerShift = -.15;

            }
        }
    }

    public double encoderAvg() {
        return (mtrFR.getCurrentPosition() + mtrFL.getCurrentPosition() + mtrBR.getCurrentPosition() + mtrBL.getCurrentPosition()) / 4;
    }

    public void setSideRoller(double Position) {
        srvRoller.setPosition(Position);
    }

<<<<<<< HEAD
    }

=======
>>>>>>> 832887b6f747ca24ab7d74bf80bfbbd26583e306
    public void gyroInches(double inches, double power) {
        reset();


        mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int distance = target(inches);
        if (distance > 0) {
            while ((mtrFR.getCurrentPosition() < distance) && (mtrFL.getCurrentPosition() < distance) &&
                    (mtrBR.getCurrentPosition() < distance) && (mtrBL.getCurrentPosition() < distance)) {
                Tank(power, power);
            }
        } else {
            while ((mtrFR.getCurrentPosition() > distance) && (mtrFL.getCurrentPosition() > distance) &&
                    (mtrBR.getCurrentPosition() > distance) && (mtrBL.getCurrentPosition() > distance)) {
                Tank(-power, -power);
            }
        }
        stopMotors();
    }

    public void PInches(double inches, double power) {
        reset();
        mtrFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int distance = target(inches);
        if (distance > 0) {
            while ((mtrFR.getCurrentPosition() < distance) && (mtrFL.getCurrentPosition() < distance) &&
                    (mtrBR.getCurrentPosition() < distance) && (mtrBL.getCurrentPosition() < distance)) {
                Tank((power * (Math.abs(distance) - Math.abs(encoderAvg()) / Math.abs(distance))),
                        (power * (Math.abs(distance) - Math.abs(encoderAvg()) / Math.abs(distance))));
            }
<<<<<<< HEAD
        } else {
            while ((mtrFR.getCurrentPosition() > distance) && (mtrFL.getCurrentPosition() > distance) &&
                    (mtrBR.getCurrentPosition() > distance) && (mtrBL.getCurrentPosition() > distance)) {
                Tank((-power * (Math.abs(distance) - Math.abs(encoderAvg()) / Math.abs(distance))),
                        (-power * (Math.abs(distance) - Math.abs(encoderAvg()) / Math.abs(distance))));
            }
        }
        stopMotors();
    }

    public void gyroInches(double inches) {
        reset();
        setMode();
        targetPosition(inches);

    }

    public void resetAngle() {
=======
            public void gyroInches ( double inches){
                    reset();
                    setMode();
                    targetPosition(inches);

    }


    public void resetAngle()
    {
>>>>>>> 832887b6f747ca24ab7d74bf80bfbbd26583e306
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.thirdAngle - lastAngles.thirdAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public double checkDirection() {
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

    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();
        //telemetry.addLine().addData("Robot Angle", getAngle());

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
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else    // left turn.
            while (getAngle() < degrees) {}

        // turn the motors off.
        stopMotors();

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
<<<<<<< HEAD



//    public void rotate(int degrees, double power){
//            double leftPower = 0, rightPower = 0;
//
//            // restart imu movement tracking.
//            resetAngle();
//            privateTelemetry.addLine().addData("Robot Angle", getAngle());
//
//            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
//            // clockwise (right).
//
//            if (degrees < 0) {   // turn right.
//                leftPower = -power;
//                rightPower = power;
//            } else if (degrees > 0) {   // turn left.
//                leftPower = power;
//                rightPower = -power;
//            }
//
//
//        // set power to rotate.
//        mtrFL.setPower(leftPower);
//        mtrBL.setPower(leftPower);
//        mtrFR.setPower(rightPower);
//        mtrBR.setPower(rightPower);
//
//        // rotate until turn is completed.
//        if (degrees < 0)
//        {
//            // On right turn we have to get off zero first.
//            while (getAngle() == 0) {}
//
//            while (getAngle() > degrees) {}
//        }
//        else    // left turn.
//            while (getAngle() < degrees) {}
//
//        // turn the motors off.
//        stopMotors();
//
//        // wait for rotation to stop.
//        sleep(1000);
//
//        // reset angle tracking on new heading.
//        resetAngle();
//    }*/
}




=======
    
}


>>>>>>> 832887b6f747ca24ab7d74bf80bfbbd26583e306
