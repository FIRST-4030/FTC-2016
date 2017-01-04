package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gyro {
    private ModernRoboticsI2cGyro gyro;
    private boolean ready = false;
    private int offset = 0;

    public Gyro(HardwareMap map, String name) {
        ready = false;
        offset = 0;
        try {
            gyro = (ModernRoboticsI2cGyro) map.gyroSensor.get(name);
            gyro.resetDeviceConfigurationForOpMode();
            gyro.calibrate();
        } catch (Exception e) {
            gyro = null;
        }
    }

    public boolean isAvailable() {
        return gyro != null;
    }

    public boolean isReady() {
        if (!ready && isAvailable() && !gyro.isCalibrating()) {
            ready = true;
        }
        return ready;
    }

    public void reset() {
        if (!isAvailable()) {
            return;
        }
        gyro.resetZAxisIntegrator();
    }

    public void setHeading(int heading) {
        setOffset(heading - getHeadingRaw());
    }

    public void setOffset(int offset) {
        this.offset = offset;
    }

    public int getHeadingRaw() {
        if (!isReady()) {
            return 0;
        }
        return gyro.getHeading();
    }

    public int getHeading() {
        return (getHeadingRaw() + offset + 360) % 360;
    }
}
