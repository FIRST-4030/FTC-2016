package org.firstinspires.ftc.teamcode.classes;

public interface DriveToListener {
    public void driveToStop(DriveToParams param);

    public void driveToRun(DriveToParams param);

    public double driveToSensor(DriveToParams param);
}
