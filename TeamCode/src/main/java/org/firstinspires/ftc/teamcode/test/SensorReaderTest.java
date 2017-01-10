package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cCompassSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by robotics on 10/12/2016.
 */

@Disabled
@TeleOp(name = "Sensor Test", group = "Test")
public class SensorReaderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Emulate a toggle button for the color sensor LED
        boolean ledButtonState;
        boolean ledButtonStateLast = false;

        // Retry timer for LED modes
        int LED_TIMEOUT = 100;
        long now = System.currentTimeMillis();
        long timeLED = now;

        // Toggles for installed sensors
        boolean enableColor, enableColor2;
        boolean enableODS;
        boolean enableGyro;
        boolean enableRange;
        boolean enableCompass;
        boolean enableIR;

        // LED state tracking
        boolean ledOn = true;

        // Init the color sensors
        ColorSensor color, color2;
        try {
            color = hardwareMap.colorSensor.get("color");
            color.enableLed(ledOn);
            enableColor = true;
        } catch (Exception e) {
            color = null;
            enableColor = false;

        }
        try {
            color2 = hardwareMap.colorSensor.get("color2");
            color2.setI2cAddress(I2cAddr.create8bit(0x1c));
            color2.enableLed(ledOn);
            enableColor2 = true;
        } catch (Exception e) {
            color2 = null;
            enableColor2 = false;
        }

        // Init the reflected light sensor
        OpticalDistanceSensor ods;
        try {
            ods = hardwareMap.opticalDistanceSensor.get("ods");
            enableODS = true;
        } catch (Exception e) {
            ods = null;
            enableODS = false;
        }

        // Init gyro
        ModernRoboticsI2cGyro gyro;
        try {
            gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
            gyro.resetDeviceConfigurationForOpMode();
            gyro.calibrate();
            enableGyro = true;
        } catch (Exception e) {
            gyro = null;
            enableGyro = false;
        }

        // Init range
        ModernRoboticsI2cRangeSensor range;
        try {
            range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
            range.initialize();
            range.enableLed(ledOn);
            enableRange = true;
        } catch (Exception e) {
            range = null;
            enableRange = false;
        }

        // Init compass
        ModernRoboticsI2cCompassSensor compass;
        try {
            compass = (ModernRoboticsI2cCompassSensor) hardwareMap.compassSensor.get("compass");
            compass.initialize();
            enableCompass = true;
        } catch (Exception e) {
            compass = null;
            enableCompass = false;
        }

        // Init IR
        IrSeekerSensor ir;
        try {
            ir = hardwareMap.irSeekerSensor.get("ir");
            enableIR = true;
        } catch (Exception e) {
            ir = null;
            enableIR = false;
        }

        // Wait for driver station start
        waitForStart();
        telemetry.clear();

        // Timer for LED flashing
        long timeLast = now;

        while (opModeIsActive()) {

            // Avoid repeated system calls
            now = System.currentTimeMillis();

            // Reset the gyro heading when A is pressed
            if (gamepad1.a || gamepad2.a) {
                if (enableGyro) {
                    gyro.resetZAxisIntegrator();
                }
            }

            // Toggle the color sensor LED when X is pressed
            ledButtonState = ledButtonStateLast;
            ledButtonStateLast = gamepad1.x | gamepad2.x;
            if ((ledButtonStateLast) && (ledButtonStateLast != ledButtonState)) {
                ledOn = !ledOn;
            }

            // Reset the LED state periodically to deal with FTC bugs
            if (timeLED + LED_TIMEOUT < now) {
                timeLED = now;
                if (enableColor) {
                    color.enableLed(ledOn);
                }
                if (enableColor2) {
                    color2.enableLed(ledOn);
                }
                if (enableRange) {
                    range.enableLed(ledOn);
                }
            }

            // Toggle the color sensor LED on a timer
            if (timeLast + 10000 < now) {
                ledOn = !ledOn;
                timeLast = now;
            }

            // Read gyro
            if (enableGyro) {
                if (gyro.isCalibrating()) {
                    telemetry.addData("Gyro", "Calibrating. DO NOT MOVE!");
                } else {
                    telemetry.addData("Heading", "%03d (Press A to reset)", gyro.getHeading());
                    telemetry.addData("Gyro", "%03d %03d %03d", gyro.rawX(), gyro.rawY(), gyro.rawZ());
                }
            }

            // Read color sensors
            if (enableColor) {
                telemetry.addData("LED", (ledOn ? "On" : "Off") + " (Press X to toggle)");
                telemetry.addData("RGB", "%03d %03d %03d", color.red(), color.green(), color.blue());
            }
            if (enableColor2) {
                telemetry.addData("LED2", (ledOn ? "On" : "Off") + " (Press X to toggle)");
                telemetry.addData("RGB2", "%03d %03d %03d", color2.red(), color2.green(), color2.blue());
            }

            // Read reflected light sensor
            if (enableODS) {
                telemetry.addData("Reflected", ods.getLightDetected());
                telemetry.addData("Reflected Raw", ods.getRawLightDetected());
            }

            // Read range
            if (enableRange) {
                telemetry.addData("Range", range.getDistance(DistanceUnit.CM));
                telemetry.addData("Range Optical", range.rawOptical());
                telemetry.addData("Range Ultrasonic", range.rawUltrasonic());
            }

            // Read compass
            if (enableCompass) {
                telemetry.addData("Compass", "%03d", compass.getDirection());
                telemetry.addData("Acceleration XYZ", "%03d %03d %03d", compass.getAcceleration().xAccel, compass.getAcceleration().yAccel, compass.getAcceleration().zAccel);
            }

            // Read IR
            if (enableIR) {
                telemetry.addData("IR", ir.signalDetected() ? "Present" : "Absent");
                telemetry.addData("IR Strength", ir.getStrength());
                telemetry.addData("IR Angle", "%03d", ir.getAngle());
            }

            // Push the driver station update
            telemetry.update();

            // Always idle() between loops
            idle();
        }
    }
}
