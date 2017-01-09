package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.classes.AllianceColor;
import org.firstinspires.ftc.teamcode.classes.DriveTo;
import org.firstinspires.ftc.teamcode.classes.DriveToComp;
import org.firstinspires.ftc.teamcode.classes.DriveToListener;
import org.firstinspires.ftc.teamcode.classes.DriveToParams;
import org.firstinspires.ftc.teamcode.classes.Gyro;
import org.firstinspires.ftc.teamcode.classes.MotorSide;
import org.firstinspires.ftc.teamcode.classes.TankDrive;
import org.firstinspires.ftc.teamcode.classes.TankMotor;
import org.firstinspires.ftc.teamcode.classes.VuforiaFTC;
import org.firstinspires.ftc.teamcode.classes.VuforiaTarget;

@SuppressWarnings("unused")
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends OpMode implements DriveToListener {

    // Field, camera and robot constants
    private static final float MM_PER_INCH = 25.4f;
    private static final int BOT_WIDTH = (int) (18 * MM_PER_INCH);
    private static final int FIELD_WIDTH = (int) ((12 * 12 - 2) * MM_PER_INCH);
    private static final float GYRO_MIN_UPDATE_INTERVAL = 1.0f;

    // Driving constants
    private static final float ENCODER_PER_MM = 3.2f;
    private static final int ENCODER_INDEX = 2;
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

    // Tracking config
    private static final String CONFIG_ASSET = "FTC_2016-17";
    private static final int CONFIG_TARGET_NUM = 4;
    // TODO: This location and rotation is imaginary, but should at least be close.
    private static final VuforiaTarget CONFIG_PHONE = new VuforiaTarget(
            "Phone", null,
            new float[]{BOT_WIDTH / 2, 0, 0},
            new float[]{-90, 0, 0},
            AxesOrder.YZY
    );
    // TODO: These locations are imaginary. We need to find the real ones for valid x/y navigation.
    private static final float[] TARGETS_ROTATION_RED = {-90, 270, 0};
    private static final float[] TARGETS_ROTATION_BLUE = {90, 0, 0};
    private static final float[] TARGETS_OFFSET_RED = {250, 0, 0};
    private static final float[] TARGETS_OFFSET_BLUE = {0, -250, 0};
    private static final int TARGETS_Y_BLUE = FIELD_WIDTH / 2;
    private static final int TARGETS_X_RED = -FIELD_WIDTH / 2;
    private static final int TARGETS_OFFSET_NEAR = (int) (12 * MM_PER_INCH);
    private static final int TARGETS_OFFSET_FAR = (int) (36 * MM_PER_INCH);
    private static final VuforiaTarget[] CONFIG = {new VuforiaTarget(
            "Wheels", AllianceColor.Color.BLUE,
            new float[]{TARGETS_OFFSET_NEAR, TARGETS_Y_BLUE, 0},
            TARGETS_OFFSET_BLUE,
            TARGETS_ROTATION_BLUE
    ), new VuforiaTarget(
            "Tools", AllianceColor.Color.RED,
            new float[]{TARGETS_X_RED, TARGETS_OFFSET_FAR, 0},
            TARGETS_OFFSET_RED,
            TARGETS_ROTATION_RED
    ), new VuforiaTarget(
            "LEGO", AllianceColor.Color.BLUE,
            new float[]{-TARGETS_OFFSET_FAR, TARGETS_Y_BLUE, 0},
            TARGETS_OFFSET_BLUE,
            TARGETS_ROTATION_BLUE
    ), new VuforiaTarget(
            "Gears", AllianceColor.Color.RED,
            new float[]{TARGETS_X_RED, -TARGETS_OFFSET_NEAR, 0},
            TARGETS_OFFSET_RED,
            TARGETS_ROTATION_RED
    )};

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
        TankMotor motors[] = new TankMotor[4];
        motors[0] = new TankMotor("fl", MotorSide.LEFT);
        motors[1] = new TankMotor("fr", MotorSide.RIGHT, true);
        motors[2] = new TankMotor("bl", MotorSide.LEFT, true); // Encoder wheel
        motors[3] = new TankMotor("br", MotorSide.RIGHT);
        tank = new TankDrive(hardwareMap, motors);
        if (!tank.isAvailable()) {
            // Note that we could retry with different names to support multiple configs/robots
            telemetry.log().add("ERROR: Unable to initalize motors");
        }

        // Vuforia
        vuforia = new VuforiaFTC(CONFIG_ASSET, CONFIG_TARGET_NUM, CONFIG, CONFIG_PHONE);
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
        telemetry.addData("Encoder", tank.getEncoder(ENCODER_INDEX));
        if (!gyro.isReady()) {
            telemetry.addData("Gyro", "Calibrating (DO NOT DRIVE): %d" + (int) time);
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
                value = tank.getEncoder(ENCODER_INDEX);
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
        param.lessThan(ticks + tank.getEncoder(ENCODER_INDEX) - OVERRUN_ENCODER);
        drive = new DriveTo(new DriveToParams[]{param});
    }
}