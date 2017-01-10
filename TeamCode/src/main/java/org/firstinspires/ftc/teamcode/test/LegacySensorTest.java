package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by robotics on 10/22/2016.
 */

@Disabled
@TeleOp(name = "Sensors: HiTech Gyro, Lego Light", group = "Sensor")
public class LegacySensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        HiTechnicNxtGyroSensor gyro;
        LightSensor lightSensor;  // Hardware Device Object
        int xVal = 0, yVal = 0, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        boolean lastResetState = false;
        boolean curResetState  = false;

        // get a reference to a Modern Robotics GyroSensor object.
        gyro = (HiTechnicNxtGyroSensor) hardwareMap.gyroSensor.get("gyro");

        /*
        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        //gyro.calibrate();


        // make sure the gyro is calibrated.
        while (gyro.isCalibrating())  {
            Thread.sleep(50);
            idle();
        }*/

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our Light Sensor object.
        lightSensor = hardwareMap.lightSensor.get("light sensor");

        // Set the LED state in the beginning.
        lightSensor.enableLed(bLedOn);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // get the x, y, and z values (rate of change of angle)

            //xVal = gyro.rawX();
            //yVal = gyro.rawY();
            //zVal = gyro.rawZ();

            // get the heading info.
            // the Modern Robotics' gyro sensor keeps
            // track of the current heading for the Z axis only.
            //heading = gyro.getHeading();

            telemetry.addData("0", "Heading %03d", heading);
            telemetry.addData("2", "X av. %03d", xVal);
            telemetry.addData("3", "Y av. %03d", yVal);
            telemetry.addData("4", "Z av. %03d", zVal);

            // check the status of the x button .
            bCurrState = gamepad1.x;

            // check for button state transitions.
            if ((bCurrState == true) && (bCurrState != bPrevState))  {

                // button is transitioning to a pressed state.  Toggle LED
                bLedOn = !bLedOn;
                lightSensor.enableLed(bLedOn);
            }

            // update previous state variable.
            bPrevState = bCurrState;

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Raw", lightSensor.getRawLightDetected());
            telemetry.addData("Normal", lightSensor.getLightDetected());

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
