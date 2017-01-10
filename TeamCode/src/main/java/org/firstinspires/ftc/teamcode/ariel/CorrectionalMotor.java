package org.firstinspires.ftc.teamcode.ariel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Ariel R-A on 8/29/2016.
 */
public abstract class CorrectionalMotor extends RegulatedMotor {

    protected double deadZone;
    protected double accumErr;
    protected double lastErr;
    protected double fullSpd;

    protected double p;
    protected double i;
    protected double d;
    protected double conversion;

    public CorrectionalMotor(DcMotorController controller, int portNumber, double deadZone, double p, double i, double d) {
        super(controller, portNumber);
        this.deadZone = deadZone;
        accumErr = 0;
        lastErr = 0;
        fullSpd = 1.0;
        this.p = p;
        this.i = i;
        this.d = d;
        conversion = 1;
    }

    public CorrectionalMotor(DcMotorController controller, int portNumber, DcMotor.Direction direction, double deadZone, double p, double i, double d) {
        super(controller, portNumber, direction);
        this.deadZone = deadZone;
        accumErr = 0;
        lastErr = 0;
        fullSpd = 1.0;
        this.p = p;
        this.i = i;
        this.d = d;
        conversion = 1;
    }

    public CorrectionalMotor(DcMotorController controller, int portNumber, double deadZone, double p, double i, double d, double conversionFactor) {
        super(controller, portNumber);
        this.deadZone = deadZone;
        accumErr = 0;
        lastErr = 0;
        fullSpd = 1.0;
        this.p = p;
        this.i = i;
        this.d = d;
        conversion = conversionFactor;
    }

    public CorrectionalMotor(DcMotorController controller, int portNumber, DcMotor.Direction direction, double deadZone, double p, double i, double d, double conversionFactor) {
        super(controller, portNumber, direction);
        this.deadZone = deadZone;
        accumErr = 0;
        lastErr = 0;
        fullSpd = 1.0;
        this.p = p;
        this.i = i;
        this.d = d;
        conversion = conversionFactor;
    }

    /**
     * Should be overridden in subclasses if extra steps should be taken during initialization
     */
    public void init() {
        this.setPower(0);
        this.setTargetPosition(0);
        //this.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        //this.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    public abstract boolean calibrate();

    /**
     * This method is for a motor that is intended to stay in one place.
     * It needs to be overridden for motors that regulate speed by encoder or gyro.
     * @return the current error
     */
    public double getError() {
        return this.getCurrentPosition() - this.getTargetPosition();
    }

    /**
     * Sets the motor power using PID
     */
    public void setPower() {
        double error = this.getError();
        accumErr += error;
        double deriv = error - lastErr;
        lastErr = error;
        double absError = Math.abs(error);
        double speed;
        if(absError <= deadZone) {
            speed = 0.0;
        } else {
            speed = (error * p + deriv * d + accumErr * i) * conversion;
        }

        if(Math.abs(speed) <= fullSpd) {
            this.setPower(speed);
        } else {
            this.setPower(fullSpd);
        }
    }

    @Override
    public boolean isOnTarget() {
        return Math.abs(getError()) <= deadZone;
    }
}
