package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wheels.MotorSide;
import org.firstinspires.ftc.teamcode.wheels.TankDrive;
import org.firstinspires.ftc.teamcode.wheels.TankMotor;

public class WheelMotorConfigs {
    private static BOT bot = null;

    public TankDrive init(HardwareMap map, Telemetry telemetry) {
        TankDrive tank = null;
        for (BOT i : BOT.values()) {
            bot = i;
            tank = new TankDrive(map, config(), encoderIndex(), encoderScale());
            if (tank.isAvailable()) {
                if (bot.ordinal() != 0) {
                    telemetry.log().add("NOTICE: Using wheel config: " + bot);
                }
                break;
            }
        }
        assert tank != null;
        if (!tank.isAvailable()) {
            telemetry.log().add("ERROR: Unable to initialize wheels");
        }
        return tank;
    }

    private TankMotor[] config() {
        TankMotor[] config = null;
        assert bot != null;
        switch (bot) {
            case FINAL:
                config = FinalBot();
                break;
            case CODE:
                config = CodeBot();
                break;
        }
        return config;
    }

    private double encoderScale() {
        double scale = 1.0;
        assert bot != null;
        switch (bot) {
            case FINAL:
                scale = FinalBotEncoderScale;
                break;
            case CODE:
                scale = CodeBotEncoderScale;
                break;
        }
        return scale;
    }

    private int encoderIndex() {
        int index = 0;
        assert bot != null;
        switch (bot) {
            case FINAL:
                index = FinalBotEncoder;
                break;
            case CODE:
                index = CodeBotEncoder;
                break;
        }
        return index;
    }

    private static final double CodeBotEncoderScale = 4.7 / 3;
    public static final int CodeBotEncoder = 2;

    public static TankMotor[] CodeBot() {
        TankMotor motors[] = new TankMotor[4];
        motors[0] = new TankMotor("fl", MotorSide.LEFT);
        motors[1] = new TankMotor("fr", MotorSide.RIGHT, true);
        motors[2] = new TankMotor("bl", MotorSide.LEFT, true);
        motors[3] = new TankMotor("br", MotorSide.RIGHT);
        return motors;
    }

    private static final double FinalBotEncoderScale = 1.0;
    private static final int FinalBotEncoder = 0;

    private static TankMotor[] FinalBot() {
        TankMotor motors[] = new TankMotor[4];
        motors[0] = new TankMotor("left front", MotorSide.LEFT);
        motors[1] = new TankMotor("right front", MotorSide.RIGHT, true);
        motors[2] = new TankMotor("left back", MotorSide.LEFT, true);
        motors[3] = new TankMotor("right back", MotorSide.RIGHT);
        return motors;
    }
}
