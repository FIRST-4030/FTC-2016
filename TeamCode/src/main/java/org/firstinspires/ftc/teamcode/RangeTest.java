package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by robotics on 11/2/2016.
 */

@TeleOp(name = "Range Test", group = "Test")
public class RangeTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ModernRoboticsI2cRangeSensor distance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasound");
        distance.initialize();
        distance.enableLed(false);

        waitForStart();
        telemetry.clear();

        while(opModeIsActive()) {
            telemetry.addData("Range", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("Range Optical CM", distance.cmOptical());
            telemetry.addData("Range Optical Raw", distance.rawOptical());
            telemetry.update();
        }
    }
}
