package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain extends robotPart {
    //motors
    public DcMotor mtrFL = null;
    public DcMotor mtrFR = null;
    public DcMotor mtrBL = null;
    public DcMotor mtrBR = null;
    public ElapsedTime runtime = new ElapsedTime();
    double     COUNTS_PER_MOTOR_REV    = 1120 ;
    double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * Math.PI);


    //sensors


    public void init(HardwareMap ahwmap, Telemetry myTelemetry){
        super.init(ahwmap, myTelemetry);

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
        mtrFL.setTargetPosition((int) (mtrFL.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        mtrFR.setTargetPosition((int) (mtrFR.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        mtrBL.setTargetPosition((int) (mtrBL.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        mtrBR.setTargetPosition((int) (mtrBR.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
    }
    public double target(double inches) {
        return (mtrBR.getCurrentPosition() + (inches * COUNTS_PER_INCH));
    }
    public void move(double power){
        mtrFL.setPower(power);
        mtrFR.setPower(power);
        mtrBL.setPower(power);
        mtrBR.setPower(power);
    }
    public void Turn(double power){
        mtrFL.setPower(-power);
        mtrBL.setPower(-power);
        mtrFR.setPower(power);
        mtrBR.setPower(power);
    }
    public void Tank(double leftPower, double rightPower) {
        mtrFL.setPower(leftPower);
        mtrBL.setPower(leftPower);
        mtrFR.setPower(rightPower);
        mtrBR.setPower(rightPower);
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
    public void goDistance(double inches, double speed){
        reset();
        setMode();
        while ((Math.abs(mtrBL.getCurrentPosition()) < target(inches))) {
            move(speed);
        }
        stopMotors();
        reset();
    }


}
