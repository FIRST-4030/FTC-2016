package org.firstinspires.ftc.teamcode.classes;

public enum MotorSide {
    LEFT(0), RIGHT(1);
    private int value;

    MotorSide(int value) {
        this.value = value;
    }

    public int get() {
        return value;
    }
}
