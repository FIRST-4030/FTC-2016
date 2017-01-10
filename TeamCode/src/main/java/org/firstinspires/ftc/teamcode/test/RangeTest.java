package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.sensors.Range;
import org.firstinspires.ftc.teamcode.wheels.MotorSide;
import org.firstinspires.ftc.teamcode.wheels.TankDrive;
import org.firstinspires.ftc.teamcode.wheels.TankMotor;

@Disabled
@SuppressWarnings("unused")
@TeleOp(name = "Range Test", group = "Test")
class RangeTest extends OpMode {

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
        TankMotor motors[] = new TankMotor[4];
        motors[0] = new TankMotor("fl", MotorSide.LEFT);
        motors[1] = new TankMotor("fr", MotorSide.RIGHT, true);
        motors[2] = new TankMotor("bl", MotorSide.LEFT, true);
        motors[3] = new TankMotor("br", MotorSide.RIGHT);
        tank = new TankDrive(hardwareMap, motors);
        if (!tank.isAvailable()) {
            telemetry.log().add("ERROR: Unable to initalize motors");
        }

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
