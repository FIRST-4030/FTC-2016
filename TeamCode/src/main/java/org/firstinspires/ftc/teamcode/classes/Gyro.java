package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Gyro {
    private ModernRoboticsI2cGyro gyro;
    private boolean ready = false;

    public Gyro(HardwareMap map, String name) {
        ready = false;
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
        if (gyro != null) {
            gyro.resetZAxisIntegrator();
        }
    }

    public int getHeading() {
        if (!isReady()) {
            return 0;
        }
        return gyro.getHeading();
    }
}
