package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.teamcode.config.WheelMotorConfigs;
import org.firstinspires.ftc.teamcode.sensors.Color;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.sensors.Range;
import org.firstinspires.ftc.teamcode.wheels.TankDrive;

@SuppressWarnings("unused")
@TeleOp(name = "CodeBot Teleop", group = "TeleopTest")
public class CodeBotTeleop extends OpMode {

    private Range range;
    private Gyro gyro;
    private Color color;
    private TankDrive tank;

    @Override
    public void init() {
        // Rangefinder
        range = new Range(hardwareMap, "range");
        if (!range.isAvailable()) {
            telemetry.log().add("ERROR: Unable to initalize rangefinder");
        }
        range.setLED(true);

        // Gyro
        gyro = new Gyro(hardwareMap, "gyro");
        if (!gyro.isAvailable()) {
            telemetry.log().add("ERROR: Unable to initalize gyroscope");
        }

        // Color
        color = new Color(hardwareMap, "color-1c", I2cAddr.create8bit(0x1c));
        if (!color.isAvailable()) {
            telemetry.log().add("Unable to init color sensor");
        }

        // Drive motors
        tank = new WheelMotorConfigs().init(hardwareMap, telemetry);
        tank.stop();

        telemetry.update();
    }

    @Override
    public void start() {
        // Allow driver control
        tank.setTeleop(true);
    }

    @Override
    public void loop() {

        // Tank drive
        tank.loop(gamepad1);

        // Encoders
        String encoders = "";
        for (int i = 0; i < tank.numMotors(); i++) {
            if (!encoders.isEmpty()) {
                encoders += "/";
            }
            encoders += tank.getEncoder(i);
        }
        telemetry.addData("Encoders", encoders);

        // Rangefinder
        range.setLED(true);
        telemetry.addData("Range", range.getRange());
        telemetry.addData("Range Optical", "%d", range.getRangeOptical());
        telemetry.addData("Range Ultrasound", "%d", range.getRangeUltrasound());

        // Color Sensor
        color.setLED(false);
        telemetry.addData("RBG", "%d %d %d", color.red(), color.blue(), color.green());
        telemetry.addData("Alpha", color.alpha());

        // Gyro
        if (gamepad1.a || gamepad2.a) {
            gyro.reset();
        }
        if (!gyro.isReady()) {
            telemetry.addData("Heading", "Calibrating " + System.currentTimeMillis());
        } else {
            telemetry.addData("Heading", "%03d (Press A to reset)", gyro.getHeading());
        }

        // Loop invariants
        telemetry.update();
    }
}
