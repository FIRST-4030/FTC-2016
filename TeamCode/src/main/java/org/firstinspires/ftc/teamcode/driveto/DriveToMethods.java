package org.firstinspires.ftc.teamcode.driveto;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by robotics on 12/2/2016.
 */

public abstract class DriveToMethods {

    private long startTime;
    private boolean isRunning;
    private long timeOutMillis = 10000;
    protected final DcMotor[] allLeftMotors;
    protected final DcMotor[] allRightMotors;

    public DriveToMethods(DcMotor[] leftMotors, DcMotor[] rightMotors) {
        allLeftMotors = leftMotors;
        allRightMotors = rightMotors;
        isRunning = false;
    }

    public void setTimeOutMillis(long millis) {
        timeOutMillis = millis;
    }

    public long getTimeOutMillis() {
        return timeOutMillis;
    }

    public boolean hasTimedOut() {
        return System.currentTimeMillis() - startTime > timeOutMillis;
    }

    public boolean driveToCondition(double leftPower, double rightPower) {
        if(!isRunning) {
            isRunning = true;
            startTime = System.currentTimeMillis();
        }

        if(hasTimedOut()) {
            stopDriveMotors();
            isRunning = false;
            return false;
        }

        if(!isOnTarget()) {
            setMotorPower(leftPower, rightPower);
            return false;
        } else {
            stopDriveMotors();
            isRunning = false;
            return true;
        }
    }

    protected abstract boolean isOnTarget();

    protected void setMotorPower(double leftPower, double rightPower) {
        for(DcMotor left: allLeftMotors) {
            left.setPower(leftPower);
        }

        for(DcMotor right: allRightMotors) {
            right.setPower(rightPower);
        }
    }

    protected void stopDriveMotors() {
        setMotorPower(0,0);
    }

}
