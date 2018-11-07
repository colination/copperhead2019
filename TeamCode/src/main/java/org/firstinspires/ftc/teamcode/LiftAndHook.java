package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LiftAndHook extends robotPart {
    //Motors
    public DcMotor mtrLiftR;
    public DcMotor mtrLiftL;
    //Servos
    public Servo servoDepositR;
    public Servo servoDepositL;
    //Variables and other useful stuff
    double encoders = 1120;
    double gearReduction = 1;
    double spoolDiameter = 1.5;
    double countsPerInch = (encoders * gearReduction)/(spoolDiameter*Math.PI);
    ElapsedTime runtime= new ElapsedTime();
    //Sensors
    public DistanceSensor sensorDistanceR;
    public DistanceSensor sensorDistanceL;
    public ColorSensor sensorColorR;
    public ColorSensor sensorColorL;
   public void init(HardwareMap ahwmap, Telemetry myTelemetry) {
        super.init(ahwmap, myTelemetry);
        mtrLiftL = ahwmap.dcMotor.get("mtrLiftL");
        mtrLiftR = ahwmap.dcMotor.get("mtrLiftR");

        mtrLiftR.setDirection(DcMotorSimple.Direction.REVERSE);

        mtrLiftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrLiftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoDepositR = ahwmap.servo.get("servoDepositR");
        servoDepositL = ahwmap.servo.get("servoDepositL");

        sensorColorR = ahwmap.colorSensor.get("sensorColorR");
        sensorColorL = ahwmap.colorSensor.get("sensorColorL");
        sensorDistanceR = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorColorR");
        sensorDistanceL = (DistanceSensor) ahwmap.opticalDistanceSensor.get("sensorColorL");
    }

    public void stop(){
       mtrLiftR.setPower(0);
       mtrLiftL.setPower(0);
    }

    public void reset() {
        mtrLiftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrLiftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMode(){
       mtrLiftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       mtrLiftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void targetPosition(double inches){
       double lTargetPosition = mtrLiftL.getCurrentPosition() + (inches * countsPerInch);
       double rTargetPosition = mtrLiftR.getCurrentPosition() + (inches * countsPerInch);

       mtrLiftL.setTargetPosition((int) lTargetPosition);
       mtrLiftR.setTargetPosition((int) rTargetPosition);
    }

    public void move(double power){
       mtrLiftR.setPower(power);
       mtrLiftL.setPower(power);
    }

    public void timeoutExit(double seconds){
        runtime.reset();
        while (runtime.seconds() < seconds && (mtrLiftL.isBusy() && mtrLiftR.isBusy())){
            privateTelemetry.addData("Path1", "Running to target position");
            privateTelemetry.addData("Path2","Running at:",
                    mtrLiftL.getCurrentPosition(),
                    mtrLiftR.getCurrentPosition());
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
    }
}