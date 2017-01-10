package org.firstinspires.ftc.teamcode.classes;

public class MotorConfig {
    public final String name;
    public final boolean reverse;

    public MotorConfig(String name, boolean reverse) {
        this.name = name;
        this.reverse = reverse;
    }

    public MotorConfig(String name) {
        this(name, false);
    }
}
