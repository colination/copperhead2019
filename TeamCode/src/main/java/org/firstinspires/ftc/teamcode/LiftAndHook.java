package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
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

//  Variables and other useful stuff
    double encoders = 1120;
    double gearReduction = .25 * (74/34);
    double spoolDiameter = 1.5;
    double countsPerInch = (encoders * gearReduction)/(spoolDiameter*Math.PI);
    ElapsedTime runtime= new ElapsedTime();

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
        //mtrLift2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Run without encoder ticks
        mtrLift1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrLift2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrLift3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servo for ball shifter
        srvShift = ahwmap.servo.get("srvShift");

        // Zero power to hold motor position
        mtrLift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLift3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoDepositR = ahwmap.servo.get("servoDepositR");
        servoDepositL = ahwmap.servo.get("servoDepositL");

        sensorColorR = ahwmap.colorSensor.get("sensorColorR");
        sensorColorL = ahwmap.colorSensor.get("sensorColorL");

        sensorDistanceR = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorColorR");
        sensorDistanceL = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorColorL");

    }

    public void stop(){
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
}