package org.firstinspires.ftc.teamcode.classes;

public class DriveToParams {
    public double limit1;
    public double limit2;
    public DriveToComp comparator;
    public final DriveToListener parent;
    public final Object reference;
    public final int timeout;

    public DriveToParams(DriveToListener parent) {
        this(parent, null);
    }

    public DriveToParams(DriveToListener parent, Object reference) {
        limit1 = 0.0f;
        limit2 = 0.0f;
        this.comparator = DriveToComp.LESS;
        this.parent = parent;
        this.reference = reference;
        this.timeout = DriveTo.TIMEOUT_DEFAULT;
    }

    public void lessThan(double limit) {
        this.comparator = DriveToComp.LESS;
        this.limit1 = limit;
        this.limit2 = limit;
    }

    public void lessThan(int limit) {
        lessThan((double) limit);
    }

    public void greaterThan(double limit) {
        this.comparator = DriveToComp.GREATER;
        this.limit1 = limit;
        this.limit2 = limit;
    }

    public void greaterThan(int limit) {
        greaterThan((double) limit);
    }

    // And so on
    // Setters/getters are optional since the members are public but might make use easier
}