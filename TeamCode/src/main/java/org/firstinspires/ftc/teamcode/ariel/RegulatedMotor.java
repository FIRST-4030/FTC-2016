package org.firstinspires.ftc.teamcode.ariel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public abstract class RegulatedMotor implements DcMotor {

    public RegulatedMotor(DcMotorController controller, int portNumber) {
        //super(controller, portNumber);
    }

    public RegulatedMotor(DcMotorController controller, int portNumber, DcMotor.Direction direction) {
        //super(controller, portNumber, direction);
    }

    public abstract boolean isOnTarget();

}