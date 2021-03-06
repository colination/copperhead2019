package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftAndHook extends robotPart {
    //Motors
    public DcMotor mtrLift1;
    public DcMotor mtrLift2;
    public DcMotor mtrLift3;
    public DcMotor mtrFlop;

    //Servos
    public Servo servoDepositR;
    public Servo servoDepositL;

    //Servos
    public  Servo srvShift;

    //Servos
    public CRServo csrvPin;


    //  Variables and other useful stuff
    double encoders = 1120;
    double gearReduction = .25 * (74/34);
    double spoolDiameter = 1.5;
    double countsPerInch = (encoders * gearReduction)/(spoolDiameter*Math.PI);
    ElapsedTime runtime= new ElapsedTime();

    double armRotate = (encoders * 7.5)/360;

    //Sensors
    public DistanceSensor sensorDistanceR;
    public DistanceSensor sensorDistanceL;
    public ColorSensor sensorColorR;
    public ColorSensor sensorColorL;
    public AnalogInput potentiometer;

   public void init(HardwareMap ahwmap, Telemetry myTelemetry) {
        super.init(ahwmap, myTelemetry);
        
        //lift Motors
        mtrLift1 = ahwmap.dcMotor.get("mtrLift1");
        mtrLift2 = ahwmap.dcMotor.get("mtrLift2");
        mtrLift3 = ahwmap.dcMotor.get("mtrLift3");

        // Rotate motor
        mtrFlop = ahwmap.dcMotor.get("mtrFlop");

        // Middle motor is reversed
        mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Run without encoder ticks
        mtrLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrLift3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servo for ball shifter
        srvShift = ahwmap.servo.get("srvShift");
        csrvPin = ahwmap.crservo.get("csrvPin");

       // Zero power to hold motor position        servoDepositL = ahwmap.servo.get("servoDepositL");
        mtrLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFlop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

   }
    public void stop() {
        mtrLift1.setPower(0);
        mtrLift2.setPower(0);
        mtrLift3.setPower(0);
    }

    public void reset() {
        mtrLift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLift3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMode(){
       mtrLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       mtrLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       mtrLift3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void targetPosition(double inches){
       double targetPosition1 = mtrLift1.getCurrentPosition() + (inches * countsPerInch);
       double targetPosition2 = mtrLift2.getCurrentPosition() + (inches * countsPerInch);
       double targetPosition3 = mtrLift3.getCurrentPosition() + (inches * countsPerInch);

       mtrLift1.setTargetPosition((int) targetPosition1);
       mtrLift2.setTargetPosition((int) targetPosition2);
       mtrLift3.setTargetPosition((int) targetPosition3);
    }
    public void armDegreePosition(double degrees){
       double targetDegrees = degrees*armRotate;
       mtrFlop.setTargetPosition((int) degrees);
       
    }
    public void timedRun() {
       mtrLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       mtrLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       mtrLift3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void move(double power){
       mtrLift1.setPower(power);
       mtrLift2.setPower(power);
       mtrLift3.setPower(power);
    }

    public void timeoutExit(double seconds){
        runtime.reset();
        while (runtime.seconds() < seconds && (mtrLift1.isBusy() && mtrLift2.isBusy() && mtrLift3.isBusy())){
            privateTelemetry.addData("Path1", "Running to target position");
            privateTelemetry.addData("Path2","Running at:",
                    mtrLift1.getCurrentPosition(),
                    mtrLift2.getCurrentPosition(),
                    mtrLift3.getCurrentPosition());
            privateTelemetry.update();
        }
    }

    public void goInches(double inches, double power, double timeout){
       runtime.reset();
       reset();
       setMode();
       targetPosition(inches);
       move(power);
       timeoutExit(timeout);
       stop();
    }
    
    public void unPin (double seconds) {
       runtime.reset();
       while (runtime.seconds() < seconds) {
           csrvPin.setPower(1);
       }
    }
//    public void pidDrive(double P, double I, double D, double target, DcMotor motor) {
//        double proportional, integral = 0, derivative, error, power = 0;
//        runtime.reset();.
//        double time = runtime.milliseconds();
//        double lastTime = 0;
//        double lastError = 0;
//        error = target - motor.getCurrentPosition();
//        while (error != 0 && privateOpMode.opModeIsActive()) {
//            proportional = error * P;
//            integral += error * (runtime.milliseconds() - lastTime) * I;
//            derivative = (error - lastError) / (runtime.seconds() - lastTime) * D;
//            power = proportional + integral - derivative;
//            move(power);`
//            lastTime = runtime.milliseconds();
//            lastError = error;
//
//        }
//    }

    public double angleTurned(AnalogInput potatometer){
       double voltage = potatometer.getVoltage();
       return voltage;
    }

    public void pidRotate(double P, double I, double D, double target, DcMotor motor){
        double proportional, integral = 0, derivative, error, power = 0;
        runtime.reset();
        double time = runtime.milliseconds();
        double lastTime = 0;
        double lastError = 0;
        error = target - motor.getCurrentPosition();
        while(error != 0 && privateOpMode.opModeIsActive()){
            proportional = error * P;
            integral += error * (runtime.milliseconds() - lastTime) * I;
            derivative = (error - lastError) / (runtime.seconds() - lastTime) * D;
            power = proportional + integral +        derivative;
            motor.setPower(power);
            lastTime = runtime.milliseconds();
            lastError =  error;
        }
    }
    public void depositMacro(){
        mtrLift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrLift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrLift3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrLift1.setTargetPosition(2000);
        mtrLift2.setTargetPosition(2000);
        mtrLift3.setTargetPosition(2000);
        mtrLift1.setPower(.4);
        mtrLift2.setPower(.4);
        mtrLift3.setPower(.4);
        if (Math.abs(mtrLift1.getCurrentPosition() - 2000) > 5){
            mtrLift1.setPower(0);
            mtrLift2.setPower(0);
            mtrLift3.setPower(0);
            mtrLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtrLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtrLift3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}