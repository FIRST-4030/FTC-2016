package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by robotics on 10/28/2016.
 */

public abstract class AutoHitOffBall extends FourWheelAutoMethods {

    public static final int GOAL_POSITION = 100;

    public void runOpMode() throws InterruptedException {
        initMotors("front left", "back left", "front right", "back right");
        while(frontLeftMotor.getCurrentPosition() < GOAL_POSITION) {
            runMotors(1.0, 1.0);
        }
        stopMotors();
    }
}
