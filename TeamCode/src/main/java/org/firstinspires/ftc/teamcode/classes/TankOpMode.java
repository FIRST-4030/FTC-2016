package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by User on 9/24/2016.
 */
public abstract class TankOpMode extends OpMode {

    protected String frontLeftMotorName;
    protected String frontRightMotorName;
    protected String backLeftMotorName;
    protected String backRightMotorName;
    protected DcMotor frontLeftMotor;
    protected DcMotor frontRightMotor;
    protected DcMotor backLeftMotor;
    protected DcMotor backRightMotor;
    protected boolean hasTwoMotors;

    public TankOpMode (String lMotorName, String rMotorName){

        frontLeftMotorName = lMotorName;
        frontRightMotorName = rMotorName;
        hasTwoMotors = true;

    }

    public TankOpMode (String fLMotor, String fRMotor, String bLMotor, String bRMotor){

        frontLeftMotorName = fLMotor;
        frontRightMotorName = fRMotor;
        backLeftMotorName = bLMotor;
        backRightMotorName = bRMotor;
        hasTwoMotors = false;
    }

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.dcMotor.get(frontLeftMotorName);
        frontRightMotor = hardwareMap.dcMotor.get(frontRightMotorName);
        if(!hasTwoMotors) {
            backLeftMotor = hardwareMap.dcMotor.get(backLeftMotorName);
            backRightMotor = hardwareMap.dcMotor.get(backRightMotorName);
        }

        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        if(!hasTwoMotors){
            backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    @Override
    public void loop() {

        CollectTelemetry();

        float left = gamepad1.left_stick_y;
        float right = gamepad1.right_stick_y;
        right = moderateMotorPower(Range.clip(right, -1f, 1f));
        left = moderateMotorPower(Range.clip(left, -1f, 1f));
        frontRightMotor.setPower(right);
        frontLeftMotor.setPower(left);
        if(!hasTwoMotors){
            backRightMotor.setPower(right);
            backLeftMotor.setPower(left);
        }

    }

    public float moderateMotorPower(float motorPower){

        if( motorPower < 0.1 && motorPower > -0.1) {

            return 0;
        }

        else{
            return motorPower;
        }
    }

    protected void CollectTelemetry(){
        telemetry.addData("G1LY", gamepad1.left_stick_y);
        telemetry.addData("G1RY", gamepad1.right_stick_y);
    }
}
