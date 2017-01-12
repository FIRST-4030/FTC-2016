package org.firstinspires.ftc.teamcode.ariel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public abstract class PositionalMotor extends RegulatedMotor {

    public PositionalMotor(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
    }

    public PositionalMotor(DcMotorController controller, int portNumber, DcMotor.Direction direction) {
        super(controller, portNumber, direction);
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

    public void setTargetPosition(int position) {
        //super.setTargetPosition(position);

    }

    public boolean isOnTarget() {
        return this.getCurrentPosition() <= this.getTargetPosition();
    }

    public abstract void runToTarget();
}