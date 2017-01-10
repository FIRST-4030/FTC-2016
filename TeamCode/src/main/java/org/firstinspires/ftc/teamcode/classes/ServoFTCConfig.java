package org.firstinspires.ftc.teamcode.classes;

public class ServoFTCConfig {
    public final String name;
    public final boolean reverse;
    public final Double min;
    public final Double max;

    public ServoFTCConfig(String name, boolean reverse, Double min, Double max) {
        this.name = name;
        this.reverse = reverse;
        this.min = min;
        this.max = max;
    }

    public ServoFTCConfig(String name) {
        this(name, false, 0d, 1d);
    }
}
