package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.actuators.Motor;
import org.firstinspires.ftc.teamcode.actuators.MotorConfig;

public class MotorConfigs {
    private static BOT bot = null;

    public Motor init(HardwareMap map, Telemetry telemetry, String name) {
        Motor motor = null;
        for (BOT i : BOT.values()) {
            bot = i;
            motor = new Motor(map, config(name));
            if (motor.isAvailable()) {
                if (bot.ordinal() != 0) {
                    telemetry.log().add("NOTICE: Using " + name + " motor config " + bot);
                }
                break;
            }
        }
        assert motor != null;
        if (!motor.isAvailable()) {
            telemetry.log().add("ERROR: Unable to initialize motor: " + name);
        }
        return motor;
    }

    private MotorConfig config(String name) {
        MotorConfig config = null;
        assert bot != null;
        switch (bot) {
            case FINAL:
                config = FinalBot(name);
                break;
            case CODE:
                config = CodeBot(name);
                break;
        }
        return config;
    }

    private static MotorConfig CodeBot(String name) {
        MotorConfig config = null;
        switch (name) {
            case "SHOOTER":
                config = new MotorConfig("shooter", false);
                break;
        }
        return config;
    }

    private static MotorConfig FinalBot(String name) {
        MotorConfig config = null;
        switch (name) {
            case "SHOOTER":
                config = new MotorConfig("shooter", false);
                break;
        }
        return config;
    }
}
