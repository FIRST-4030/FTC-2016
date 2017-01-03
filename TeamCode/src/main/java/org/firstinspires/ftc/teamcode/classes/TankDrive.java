package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TankDrive {
    private TankMotor[] motors = null;
    private boolean disabled = true;

    public TankDrive(HardwareMap map, TankMotor[] motors) {
        try {
            this.motors = motors;
            for (TankMotor motor : this.motors) {
                motor.motor = map.dcMotor.get(motor.name);
                if (motor.reverse) {
                    motor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
                }
            }
            this.disabled = false;
        } catch (Exception e) {
            this.motors = null;
            this.disabled = true;
        }
    }

    public boolean isAvailable() {
        return motors != null;
    }

    public void setSpeed(double speed, String name) {
        if (isDisabled()) {
            return;
        }
        for (TankMotor motor : motors) {
            if (motor.name.equals(name)) {
                motor.motor.setPower(speed);
            }
        }
    }

    public void setSpeed(double speed, MotorSide side) {
        if (isDisabled()) {
            return;
        }
        for (TankMotor motor : motors) {
            if (motor.side == side) {
                motor.motor.setPower(speed);
            }
        }
    }

    public void stop() {
        for (TankMotor motor : motors) {
            motor.motor.setPower(0.0d);
        }
    }

    public boolean isDisabled() {
        return this.disabled;
    }

    public void setDisabled(boolean disabled) {
        if (!this.disabled && disabled) {
            this.stop();
        }
        this.disabled = disabled;
    }

    public void loop(Gamepad pad) {
        float left = cleanJoystick(pad.left_stick_y);
        this.setSpeed(left, MotorSide.LEFT);

        float right = cleanJoystick(pad.right_stick_y);
        this.setSpeed(right, MotorSide.RIGHT);
    }

    private float cleanJoystick(float power) {
        power = com.qualcomm.robotcore.util.Range.clip(power, -1f, 1f);
        if (power < 0.1 && power > -0.1) {
            return 0;
        } else {
            return power;
        }
    }
}
