package org.firstinspires.ftc.teamcode.actuators;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoFTC {
    private Servo servo;
    private Double min;
    private Double max;

    public ServoFTC(HardwareMap map, ServoFTCConfig config) {
        try {
            servo = map.servo.get(config.name);
            if (config.reverse) {
                servo.setDirection(Servo.Direction.REVERSE);
            }
            this.min = config.min;
            this.max = config.max;
        } catch (Exception e) {
            servo = null;
        }
    }

    public boolean isAvailable() {
        return servo != null;
    }

    public void setPosition(double position) {
        if (min != null && position < min) {
            position = min;
        } else if (max != null && position > max) {
            position = max;
        }
        setPositionRaw(position);
    }

    public void setPositionRaw(double position) {
        if (!isAvailable()) {
            return;
        }
        servo.setPosition(position);
    }

    public void min() {
        if (min != null) {
            setPosition(min);
        }
    }

    public void max() {
        if (max != null) {
            setPosition(max);
        }
    }
}
