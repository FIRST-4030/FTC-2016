package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.WheelMotorConfigs;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.sensors.Range;
import org.firstinspires.ftc.teamcode.wheels.TankDrive;

@SuppressWarnings("unused")
@TeleOp(name = "CodeBot Teleop", group = "TeleopTest")
class CodeBotTeleop extends OpMode {

    private Range range;
    private Gyro gyro;
    private TankDrive tank;

    @Override
    public void init() {
        telemetry.clearAll();

        // Sensors
        range = new Range(hardwareMap, "range");
        if (!range.isAvailable()) {
            telemetry.log().add("ERROR: Unable to initalize rangefinder");
        }
        gyro = new Gyro(hardwareMap, "gyro");
        if (!gyro.isAvailable()) {
            telemetry.log().add("ERROR: Unable to initalize gyroscope");
        }

        // Drive motors
        tank = new WheelMotorConfigs().init(hardwareMap, telemetry);

        telemetry.update();
    }

    @Override
    public void start() {
        // Allow driver control
        tank.stop();
        tank.setTeleop(true);
    }

    @Override
    public void loop() {

        tank.loop(gamepad1);
        telemetry.addData("Encoder 0/1/2/3", tank.getEncoder(0) + "/" + tank.getEncoder(1) + "/" +
                tank.getEncoder(2) + "/" + tank.getEncoder(3));

        range.setLED(true);
        telemetry.addData("Range", range.getRange());
        telemetry.addData("Range Optical", "%d", range.getRangeOptical());
        telemetry.addData("Range Ultrasound", "%d", range.getRangeUltrasound());

        if (gamepad1.a || gamepad2.a) {
            gyro.reset();
        }
        if (!gyro.isReady()) {
            telemetry.addData("Heading", "Calibrating " + System.currentTimeMillis());
        } else {
            telemetry.addData("Heading", "%03d (Press A to reset)", gyro.getHeading());
        }

        telemetry.update();
    }
}
