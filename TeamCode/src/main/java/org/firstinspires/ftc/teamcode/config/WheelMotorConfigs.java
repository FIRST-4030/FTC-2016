package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.wheels.MotorSide;
import org.firstinspires.ftc.teamcode.wheels.TankMotor;

public class WheelMotorConfigs {
    public static final int CodeBotEncoder = 2;
    public static TankMotor[] CodeBot() {
        TankMotor motors[] = new TankMotor[4];
        motors[0] = new TankMotor("fl", MotorSide.LEFT);
        motors[1] = new TankMotor("fr", MotorSide.RIGHT, true);
        motors[2] = new TankMotor("bl", MotorSide.LEFT, true);
        motors[3] = new TankMotor("br", MotorSide.RIGHT);
        return motors;
    }

    public static final int FinalBotEncoder = 0;
    public static TankMotor[] FinalBot() {
        TankMotor motors[] = new TankMotor[4];
        motors[0] = new TankMotor("left front", MotorSide.LEFT);
        motors[1] = new TankMotor("right front", MotorSide.RIGHT, true);
        motors[2] = new TankMotor("left back", MotorSide.LEFT, true);
        motors[3] = new TankMotor("right back", MotorSide.RIGHT);
        return motors;
    }
}
