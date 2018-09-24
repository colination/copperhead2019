package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TrollBotHW {

    HardwareMap hwMap           =  null;
    ElapsedTime runtime = new ElapsedTime();
    // Drivetrain drivetrain = anew drivetrain();
    public DcMotor  motorFL  = null;
    public DcMotor  motorFR  = null;
    public DcMotor  motorBL  = null;
    public DcMotor  motorBR  = null;

    double     COUNTS_PER_MOTOR_REV    = 1440 ;
    double     DRIVE_GEAR_REDUCTION    = 2.6 ;
    double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/(WHEEL_DIAMETER_INCHES * 3.1415);





    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //drivetrain.init(Telemetry8)

        // Define and Initialize Motors
        motorFL = hwMap.get(DcMotor.class, "motorFL");
        motorFR = hwMap.get(DcMotor.class, "motorFR");
        motorBL = hwMap.get(DcMotor.class, "motorBL");
        motorBR = hwMap.get(DcMotor.class, "motorBR");

        // Set all motors to zero power
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void setMode(){
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void reset() {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
}
    public void targetPosition(double inches){
        motorFL.setTargetPosition((int) (motorFL.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        motorFR.setTargetPosition((int) (motorFL.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        motorBL.setTargetPosition((int) (motorFL.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
        motorBR.setTargetPosition((int) (motorFL.getCurrentPosition() + (inches * COUNTS_PER_INCH)));
    }
    public void move(double power){
        motorFL.setPower(power);
        motorFR.setPower(power);
        motorBL.setPower(power);
        motorBR.setPower(power);
    }
    public void fullReset(){
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void goInches(double inches, double speed, double time){
        runtime.reset();
        reset();
        setMode();
        targetPosition(inches);
        while (runtime.seconds() < time){
            move(speed);
        }
        move(0);
        fullReset();
    }

}
