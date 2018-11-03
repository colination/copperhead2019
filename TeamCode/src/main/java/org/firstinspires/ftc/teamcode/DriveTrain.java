package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain extends robotPart {
    //motors
    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DcMotor mtrBL = null;
    public DcMotor mtrBR = null;

    //servos
    public Servo srvRoller = null;

    public ElapsedTime runtime = new ElapsedTime();
    double     COUNTS_PER_MOTOR_REV    = 1120 ;
    double     DRIVE_GEAR_REDUCTION    = 1 ;
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
        //srvRoller.setPosition(-1);

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
        mtrFL.setTargetPosition((int) (mtrFL.getCurrentPosition() + (inches * 2 * COUNTS_PER_INCH)));
        mtrFR.setTargetPosition((int) (mtrFR.getCurrentPosition() + (inches * 2 * COUNTS_PER_INCH)));
        mtrBL.setTargetPosition((int) (mtrBL.getCurrentPosition() + (inches * 2 * COUNTS_PER_INCH)));
        mtrBR.setTargetPosition((int) (mtrBR.getCurrentPosition() + (inches * 2 * COUNTS_PER_INCH)));
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
    public void turnMode(){
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

    public void goLean(double inches, double power, double timeout, boolean direction){
        double powerShift;
        //true for direction is forward, false for direction is backwards
        if (direction == true){
            powerShift = .1;
        }
        else{
            powerShift = -.1;
        }
        runtime.reset();
        reset();
        setMode();
        targetPosition(inches);
        moveLean(power,powerShift);
        timeoutExit(timeout);
        stopMotors();
        reset();
    }

    public void gyroInches(double inches){
        reset();
        setMode();
        targetPosition(inches);
    }
}
