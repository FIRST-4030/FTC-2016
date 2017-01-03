package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Range {
    private ModernRoboticsI2cRangeSensor range;

    public Range(HardwareMap map, String name) {
        try {
            range = map.get(ModernRoboticsI2cRangeSensor.class, name);
            range.initialize();
            range.enableLed(true);
        } catch (Exception e) {
            range = null;
        }
    }

    public boolean isAvailable() {
        return range != null;
    }

    public int getRange() {
        if (!isAvailable()) {
            return 0;
        }
        return (int) range.getDistance(DistanceUnit.CM);
    }

    public int getRangeOptical() {
        if (!isAvailable()) {
            return 0;
        }
        return (int) range.cmOptical();
    }

    public int getRangeUltrasound() {
        if (!isAvailable()) {
            return 0;
        }
        return (int) range.cmUltrasonic();
    }

    public void setLED(boolean state) {
        if (isAvailable()) {
            range.enableLed(state);
        }
    }
}
