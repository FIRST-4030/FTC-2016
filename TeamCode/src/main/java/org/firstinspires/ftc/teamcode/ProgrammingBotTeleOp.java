package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.teamcode.classes.TankOpMode;

/**
 * Created by robotics on 11/5/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tank Tele-Op", group = "Iterative Opmode")
public class ProgrammingBotTeleOp extends TankOpMode {

    ModernRoboticsI2cRangeSensor range;
    ColorSensor color;
    LightSensor light;

    public ProgrammingBotTeleOp() {
        super("front left", "front right", "back left", "back right");
    }

    public void init() {
        super.init();

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasound");
        color = hardwareMap.colorSensor.get("color sensor1");
        light = hardwareMap.lightSensor.get("light");

        color.enableLed(true);
        light.enableLed(true);

        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {
        super.loop();
        telemetry.update();
    }

    protected void CollectTelemetry() {
        telemetry.addData("Light Sensor:", light.getRawLightDetected());
        telemetry.addData("RGB", "%03d %03d %03d", color.red(), color.green(), color.blue());
        telemetry.addData("Range Optical Raw", range.rawOptical());
    }
}
