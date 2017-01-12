package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.driveto.DriveTo;
import org.firstinspires.ftc.teamcode.driveto.DriveToComp;
import org.firstinspires.ftc.teamcode.driveto.DriveToListener;
import org.firstinspires.ftc.teamcode.driveto.DriveToParams;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.wheels.MotorSide;
import org.firstinspires.ftc.teamcode.wheels.TankDrive;
import org.firstinspires.ftc.teamcode.vuforia.VuforiaFTC;
import org.firstinspires.ftc.teamcode.config.VuforiaConfigs;
import org.firstinspires.ftc.teamcode.config.WheelMotorConfigs;

@SuppressWarnings("unused")
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends OpMode implements DriveToListener {

    // Driving constants
    private static final float GYRO_MIN_UPDATE_INTERVAL = 1.0f;
    private static final float ENCODER_PER_MM = 3.2f;
    private static final float SPEED_TURN = 0.1f;
    private static final float SPEED_TURN_FAST = 0.5f;
    private static final int SPEED_TURN_THRESHOLD = 60;
    private static final float SPEED_DRIVE = 1.0f;
    private static final int TIMEOUT_DEFAULT = DriveTo.TIMEOUT_DEFAULT;
    private static final int TIMEOUT_DEGREE = 100;
    private static final int OVERRUN_GYRO = 2;
    private static final int OVERRUN_ENCODER = 25;

    // Numeric constants
    private final static int FULL_CIRCLE = 360;

    // Dynamic things we need to remember
    private VuforiaFTC vuforia;
    private TankDrive tank;
    private Gyro gyro;
    private DriveTo drive;
    private double headingSyncExpires;
    private int lastBearing = 0;
    private int lastDistance = 0;
    private String lastTarget = "<None>";

    // Sensor reference types for our DriveTo callbacks
    enum SENSOR_TYPE {
        GYRO, ENCODER
    }

    @Override
    public void init() {

        // Placate drivers; sometimes VuforiaFTC is slow to init
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        // Sensors
        gyro = new Gyro(hardwareMap, "gyro");
        if (!gyro.isAvailable()) {
            telemetry.log().add("ERROR: Unable to initalize gyro");
        }

        // Drive motors
        tank = new TankDrive(hardwareMap, WheelMotorConfigs.CodeBot(), WheelMotorConfigs.CodeBotEncoder);
        if (!tank.isAvailable()) {
            // Note that we could retry with different names to support multiple configs/robots
            telemetry.log().add("ERROR: Unable to initalize motors");
        }

        // Vuforia
        vuforia = new VuforiaFTC(VuforiaConfigs.AssetName, VuforiaConfigs.TargetCount,
                VuforiaConfigs.Field(), VuforiaConfigs.Bot());
        vuforia.init();

        // Wait for the game to begin
        telemetry.addData(">", "Ready for game start");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.clearAll();

        // Start Vuforia tracking
        vuforia.start();

        // Allow driver control
        tank.stop();
        tank.setTeleop(true);
    }

    @SuppressWarnings("UnnecessaryReturnStatement")
    @Override
    public void loop() {
        // Handle DriveTo driving
        if (drive != null) {
            // DriveTo
            drive.drive();

            // Return to teleop when complete
            if (drive.isDone()) {
                drive = null;
                tank.setTeleop(true);
            }
        }

        // Driver feedback
        vuforia.display(telemetry);
        telemetry.addData("Target (" + lastTarget + ")", lastDistance + "mm @ " + lastBearing + "°");
        telemetry.addData("Encoder", tank.getEncoder());
        if (!gyro.isReady()) {
            telemetry.addData("Gyro", "Calibrating (DO NOT DRIVE): %d", (int) time);
        } else {
            telemetry.addData("Gyro Abs/Rel", gyro.getHeading() + "°/" + gyro.getHeadingRaw() + "°");
        }
        telemetry.update();

        /*
         * Cut the loop short when we are auto-driving
         */
        if (drive != null) {
            return;
        }

        // Drive
        tank.loop(gamepad1);

        // Update our location and target info
        vuforia.track();

        // Update the gyro offset if we have a fix
        if (!vuforia.isStale() && headingSyncExpires < time) {
            headingSyncExpires = time + GYRO_MIN_UPDATE_INTERVAL;
            gyro.setHeading(vuforia.getHeading());
        }

        // Collect data about the first visible target
        boolean valid = false;
        String target = null;
        int bearing = 0;
        int distance = 0;
        if (!vuforia.isStale()) {
            for (String t : vuforia.getVisible().keySet()) {
                if (vuforia.getVisible(t)) {
                    target = t;
                    int index = vuforia.getTargetIndex(t);
                    bearing = vuforia.bearing(index);
                    distance = vuforia.distance(index);
                    valid = true;
                    break;
                }
            }
            lastTarget = target;
            lastBearing = bearing;
            lastDistance = distance;
        }


        /*
         * It's always safe to return after this; it should be nothing but auto-drive modes
         *
         * Typically you only want to trigger a single auto-drive mode at any given time so
         * be sure to return after selecting one
         */

        // Turn 90° left/right
        if (gamepad1.right_bumper) {
            if (gamepad1.x) {
                turnAngle(-90);
            } else if (gamepad1.b) {
                turnAngle(90);
            }
            return;
        }

        // Turn to face cardinal directions (or our best guess if we've never seen a target)
        if (gamepad1.left_bumper) {
            if (gamepad1.y) {
                turnBearing(0);
            } else if (gamepad1.b) {
                turnBearing(90);
            } else if (gamepad1.a) {
                turnBearing(180);
            } else if (gamepad1.x) {
                turnBearing(270);
            }
            return;
        }

        /*
         * Cut the loop short when we don't have a vision fix
         */
        if (!valid) {
            return;
        }

        // Turn to face the first visible target
        if (gamepad1.y) {
            turnBearing(bearing);
            return;
        }

        // Drive half the distance to the first visible target
        if (gamepad1.x) {
            driveForward(distance / 2);
            return;
        }
    }

    @Override
    public void driveToStop(DriveToParams param) {
        tank.stop();
    }

    @Override
    public void driveToRun(DriveToParams param) {
        // Remember that "forward" is "negative" per the joystick conventions
        switch ((SENSOR_TYPE) param.reference) {
            case GYRO:
                // Turn faster if we're outside the threshold
                float speed = SPEED_TURN;
                if (Math.abs(param.error1) > SPEED_TURN_THRESHOLD) {
                    speed = SPEED_TURN_FAST;
                }

                // Turning clockwise increases heading
                if (param.comparator.equals(DriveToComp.GREATER)) {
                    tank.setSpeed(-speed, MotorSide.LEFT);
                    tank.setSpeed(speed, MotorSide.RIGHT);
                } else {
                    tank.setSpeed(speed, MotorSide.LEFT);
                    tank.setSpeed(-speed, MotorSide.RIGHT);
                }
                break;
            case ENCODER:
                // Always drive forward
                tank.setSpeed(-SPEED_DRIVE);
                break;
        }
    }

    @Override
    public double driveToSensor(DriveToParams param) {
        double value = 0;
        switch ((SENSOR_TYPE) param.reference) {
            case GYRO:
                value = gyro.getHeading();
                break;
            case ENCODER:
                value = tank.getEncoder();
                break;
        }
        return value;
    }

    private void turnAngle(int angle) {
        tank.setTeleop(false);
        DriveToParams param = new DriveToParams(this, SENSOR_TYPE.GYRO);
        param.timeout = (Math.abs(angle) * TIMEOUT_DEGREE) + TIMEOUT_DEFAULT;

        // Normalized heading and bearing
        int target = gyro.getHeading() + angle;

        // Turn CCW for negative angles
        if (angle > 0) {
            param.greaterThan(target - OVERRUN_GYRO);
        } else {
            param.lessThan(target + OVERRUN_GYRO);
        }
        drive = new DriveTo(new DriveToParams[]{param});
    }

    private void turnBearing(int bearing) {
        // Normalized heading and turns in each direction
        int heading = gyro.getHeadingBasic();
        int cw = (bearing - heading + FULL_CIRCLE) % FULL_CIRCLE;
        int ccw = (heading - bearing + FULL_CIRCLE) % FULL_CIRCLE;

        // Turn the short way
        if (Math.abs(cw) <= Math.abs(ccw)) {
            turnAngle(cw);
        } else {
            turnAngle(-ccw);
        }
    }

    private void driveForward(int distance) {
        tank.setTeleop(false);
        DriveToParams param = new DriveToParams(this, SENSOR_TYPE.ENCODER);
        int ticks = (int) ((float) -distance * ENCODER_PER_MM);
        param.lessThan(ticks + tank.getEncoder() - OVERRUN_ENCODER);
        drive = new DriveTo(new DriveToParams[]{param});
    }
}