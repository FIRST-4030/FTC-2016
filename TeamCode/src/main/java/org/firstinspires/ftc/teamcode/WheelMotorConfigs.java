package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.classes.MotorSide;
import org.firstinspires.ftc.teamcode.classes.TankMotor;

public class WheelMotorConfigs {
    public static TankMotor[] CodeBot(int year) {
        TankMotor motors[] = null;

        switch(year) {
            case 2016:
                motors = new TankMotor[4];
                motors[0] = new TankMotor("fl", MotorSide.LEFT);
                motors[1] = new TankMotor("fr", MotorSide.RIGHT, true);
                motors[2] = new TankMotor("bl", MotorSide.LEFT, true); // Encoder wheel
                motors[3] = new TankMotor("br", MotorSide.RIGHT);
                break;
        }

        return motors;
    }
}
