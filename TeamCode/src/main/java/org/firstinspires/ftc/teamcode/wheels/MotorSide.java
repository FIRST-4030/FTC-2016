package org.firstinspires.ftc.teamcode.wheels;

public enum MotorSide {
    LEFT(0), RIGHT(1);
    private final int value;

    MotorSide(int value) {
        this.value = value;
    }

    public int get() {
        return value;
    }
}
