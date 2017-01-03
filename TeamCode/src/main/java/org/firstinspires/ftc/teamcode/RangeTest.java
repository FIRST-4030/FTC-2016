package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.Gyro;
import org.firstinspires.ftc.teamcode.classes.Range;
import org.firstinspires.ftc.teamcode.classes.MotorSide;
import org.firstinspires.ftc.teamcode.classes.TankDrive;
import org.firstinspires.ftc.teamcode.classes.TankMotor;

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
        tank.stop();
    }

    @Override
    public void loop() {

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

        tank.loop(gamepad1);

        telemetry.update();
    }
}
