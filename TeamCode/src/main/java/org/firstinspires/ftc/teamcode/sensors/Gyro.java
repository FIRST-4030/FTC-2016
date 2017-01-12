package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gyro {
    private static final int FULL_CIRCLE = 360;

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

    public void disable() {
        ready = false;
        gyro = null;
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
        // Normalize heading and offset
        heading = (heading + FULL_CIRCLE) % FULL_CIRCLE;
        int offset = (heading - getHeadingBasic(true)) % FULL_CIRCLE;
        if (offset > FULL_CIRCLE / 2) {
            offset -= FULL_CIRCLE;
        }
        setOffset(offset);
    }

    public void setOffset(int offset) {
        this.offset = offset;
    }

    public int getHeadingRaw() {
        if (!isReady()) {
            return 0;
        }

        // Invert to make CW rotation increase the heading
        return -gyro.getIntegratedZValue();
    }

    public int getHeading() {
        return (getHeadingRaw() + offset);
    }

    public int getHeadingBasic() {
        return getHeadingBasic(false);
    }

    private int getHeadingBasic(boolean raw) {
        int heading;
        if (raw) {
            heading = getHeadingRaw();
        } else {
            heading = getHeading();
        }
        return ((heading % FULL_CIRCLE) + FULL_CIRCLE) % FULL_CIRCLE;
    }
}