package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Color {
    private ColorSensor color;

    public Color(HardwareMap map, String name) {
        this(map, name, null);
    }

    public Color(HardwareMap map, String name, I2cAddr addr) {
        try {
            color = map.colorSensor.get(name);
            color.enableLed(true);
            if (addr != null) {
                color.setI2cAddress(addr);
            }
        } catch (Exception e) {
            color = null;
        }
    }

    public boolean isAvailable() {
        return color != null;
    }

    public int red() {
        if (!isAvailable()) {
            return 0;
        }
        return color.red();
    }

    public int green() {
        if (!isAvailable()) {
            return 0;
        }
        return color.green();
    }

    public int blue() {
        if (!isAvailable()) {
            return 0;
        }
        return color.blue();
    }

    public int alpha() {
        if (!isAvailable()) {
            return 0;
        }
        return color.alpha();
    }

    public void setLED(boolean state) {
        if (!isAvailable()) {
            return;
        }
        color.enableLed(state);
    }
}
