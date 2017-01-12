package org.firstinspires.ftc.teamcode.driveto;

public interface DriveToListener {
    void driveToStop(DriveToParams param);

    void driveToRun(DriveToParams param);

    double driveToSensor(DriveToParams param);
}
