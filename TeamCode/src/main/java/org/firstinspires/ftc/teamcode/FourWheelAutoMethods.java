package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Timer;

/**
 * Created by User on 10/29/2016.
 */
public abstract class FourWheelAutoMethods extends OpMode {

    protected DcMotor frontLeftMotor;
    protected DcMotor backLeftMotor;
    protected DcMotor frontRightMotor;
    protected DcMotor backRightMotor;
    protected Timer timer;

    public void initMotors(String frontLeft, String backLeft, String frontRight, String backRight) {
        frontLeftMotor = hardwareMap.dcMotor.get(frontLeft);
        backLeftMotor = hardwareMap.dcMotor.get(backLeft);
        frontRightMotor = hardwareMap.dcMotor.get(frontRight);
        backRightMotor = hardwareMap.dcMotor.get(backRight);
    }

    public void runMotors(double frontLeft, double backLeft, double frontRight, double backRight) {
        frontLeftMotor.setPower(frontLeft);
        backLeftMotor.setPower(backLeft);
        frontRightMotor.setPower(frontRight);
        backRightMotor.setPower(backRight);
    }

    public void runMotors(double left, double right) {
        runMotors(left, left, right, right);
    }

    public void stopMotors() {
        runMotors(0, 0);
    }

    public void driveToTime(double left, double right, long millis) throws InterruptedException {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < millis) {
            runMotors(left, right);
        }
        stopMotors();
    }

    public void robotWait(long millis) throws InterruptedException {
        timer.wait(millis);
    }

}
