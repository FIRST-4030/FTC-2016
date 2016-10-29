package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by robotics on 10/28/2016.
 */

public class AutoHitOffBall extends LinearOpMode {

    public static final int GOAL_POSITION = 100;

    @Override
    public void runOpMode() throws InterruptedException {
        //get motors
        DcMotor leftMotor;
        while(leftMotor.getCurrentPosition() < GOAL_POSITION) {
            leftMotor.setPower(1.0);
        }
        leftMotor.setPower(0);

    }
}
