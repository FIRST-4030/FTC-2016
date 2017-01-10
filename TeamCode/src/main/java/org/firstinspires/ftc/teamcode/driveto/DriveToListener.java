package org.firstinspires.ftc.teamcode.driveto;

public interface DriveToListener {
    public void driveToStop(DriveToParams param);

    public void driveToRun(DriveToParams param);

    public double driveToSensor(DriveToParams param);
}
