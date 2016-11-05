package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by robotics on 10/12/2016.
 */

@TeleOp(name = "Sensor Test", group = "Test")
public class SensorReaderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Emulate a toggle button for the color sensor LED
        boolean ledButtonState = false;
        boolean ledButtonStateLast = false;

        // Init the color sensor
        boolean ledOn = true;
        ColorSensor color = hardwareMap.colorSensor.get("color sensor");
        color.enableLed(ledOn);

        // Init the reflected light sensor
        OpticalDistanceSensor ods = hardwareMap.opticalDistanceSensor.get("ods");

        // Init gyro
        ModernRoboticsI2cGyro gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.resetDeviceConfigurationForOpMode();
        gyro.calibrate();
        while (gyro.isCalibrating())  {
            telemetry.addData("Gyro", "Calibrating. DO NOT MOVE!");
            telemetry.addData("Status", gyro.status());
            telemetry.addData("Time", System.currentTimeMillis());
            telemetry.update();
            Thread.sleep(50);
            idle();
        }
        telemetry.addData("Gyro", "Calibrated. Press start.");
        telemetry.update();

        // Init range
        ModernRoboticsI2cRangeSensor range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        range.initialize();
        range.enableLed(ledOn);

        // Init compass
        ModernRoboticsI2cCompassSensor compass = (ModernRoboticsI2cCompassSensor)hardwareMap.compassSensor.get("compass");
        compass.initialize();

        // Init IR
        IrSeekerSensor ir = hardwareMap.irSeekerSensor.get("ir");

        // Wait for driver station start
        waitForStart();
        telemetry.clear();

        while (opModeIsActive())  {

            // Reset the gyro heading when A is pressed
            if(gamepad1.a || gamepad2.a)  {
                gyro.resetZAxisIntegrator();
            }

            // Toggle the color sensor LED when X is pressed
            ledButtonState = ledButtonStateLast;
            ledButtonStateLast = gamepad1.x | gamepad2.x;
            if ((ledButtonStateLast == true) && (ledButtonStateLast != ledButtonState))  {
                ledOn = !ledOn;
                color.enableLed(ledOn);
                range.enableLed(ledOn);
            }

            // Read gyro
            telemetry.addData("Heading", "%03d (Press A to reset)", gyro.getHeading());
            telemetry.addData("Gyro XYZ", "%03d %03d %03d", gyro.rawX(), gyro.rawY(), gyro.rawZ());

            // Read color sensor
            telemetry.addData("LED", (ledOn ? "On" : "Off") + " (Press X to toggle)");
            telemetry.addData("RGB", "%03d %03d %03d", color.red(), color.green(), color.blue());

            // Read reflected light sensor
            telemetry.addData("Reflected", ods.getLightDetected());
            telemetry.addData("Reflected Raw", ods.getRawLightDetected());

            // Read range
            telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
            telemetry.addData("Range Optical", range.rawOptical());
            telemetry.addData("Range Ultrasonic", range.rawUltrasonic());

            // Read compass
            telemetry.addData("Compass", "%03d", compass.getDirection());
            telemetry.addData("Acceleration XYZ", "%03d %03d %03d", compass.getAcceleration().xAccel, compass.getAcceleration().yAccel,compass.getAcceleration().zAccel);

            // Read IR
            telemetry.addData("IR", ir.signalDetected() ? "Present" : "Absent");
            telemetry.addData("IR Strength", ir.getStrength());
            telemetry.addData("IR Angle", "%03d", ir.getAngle());

            // Push the driver station update
            telemetry.update();

            // Always idle() between loops
            idle();
        }
    }
}
