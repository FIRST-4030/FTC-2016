package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
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
    private static final int GYRO_MIN_UPDATE_INTERVAL = 5;

    // Driving constants
    private static final float ENCODER_PER_MM = 3.2f;
    private static final int ENCODER_INDEX = 2;
    private static final float SPEED_TURN = 0.5f;
    private static final float SPEED_DRIVE = 1.0f;

    // Numeric constants
    private final static int FULL_CIRCLE = 360;
    private final static int HALF_CIRCLE = FULL_CIRCLE / 2;

    // Tracking config
    private static final String CONFIG_ASSET = "FTC_2016-17";
    private static final int CONFIG_TARGET_NUM = 4;
    // TODO: This location and rotation is imaginary, but should at least be close.
    private static final VuforiaTarget CONFIG_PHONE = new VuforiaTarget(
            "Phone",
            new float[]{BOT_WIDTH / 2, 0, 0},
            new float[]{-90, 0, 0},
            AxesOrder.YZY
    );
    // TODO: These locations are imaginary. We need to find the real ones before navigation.
    private static final float[] TARGETS_ROTATION_RED = {-90, 270, 0};
    private static final float[] TARGETS_ROTATION_BLUE = {90, 0, 0};
    private static final float[] TARGETS_OFFSET_RED = {250, 0, 0};
    private static final float[] TARGETS_OFFSET_BLUE = {0, -250, 0};
    private static final int TARGETS_Y_BLUE = FIELD_WIDTH / 2;
    private static final int TARGETS_X_RED = -FIELD_WIDTH / 2;
    private static final int TARGETS_OFFSET_NEAR = (int) (12 * MM_PER_INCH);
    private static final int TARGETS_OFFSET_FAR = (int) (36 * MM_PER_INCH);
    private static final VuforiaTarget[] CONFIG = {new VuforiaTarget(
            "Wheels",
            new float[]{TARGETS_OFFSET_NEAR, TARGETS_Y_BLUE, 0},
            TARGETS_OFFSET_BLUE,
            TARGETS_ROTATION_BLUE
    ), new VuforiaTarget(
            "Tools",
            new float[]{TARGETS_X_RED, TARGETS_OFFSET_FAR, 0},
            TARGETS_OFFSET_RED,
            TARGETS_ROTATION_RED
    ), new VuforiaTarget(
            "LEGO",
            new float[]{-TARGETS_OFFSET_FAR, TARGETS_Y_BLUE, 0},
            TARGETS_OFFSET_BLUE,
            TARGETS_ROTATION_BLUE
    ), new VuforiaTarget(
            "Gears",
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

    // TODO: Debug
    private int lastBearing = 0;
    private String thisTarget = "";
    private String lastTarget = "";

    // Sensor reference types for our DriveTo callbacks
    enum SENSOR_TYPE {
        GYRO, GYRO_SECONDARY, ENCODER
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

        // TODO: Debug
        if (true) {
            // Reduce speeds while testing
            tank.setSpeedScale(0.5);
        }
    }

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
        telemetry.addData("Encoder", tank.getEncoder(ENCODER_INDEX));
        if (!gyro.isReady()) {
            telemetry.addData("Gyro", "Calibrating (DO NOT DRIVE): %d" + (int) time);
        } else {
            telemetry.addData("Gyro Abs/Rel", gyro.getHeading() + "/" + gyro.getHeadingRaw());
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
        int bearing = 0;
        int distance = 0;
        if (!vuforia.isStale()) {
            for (String target : vuforia.getVisible().keySet()) {
                if (vuforia.getVisible(target)) {
                    int index = vuforia.getTargetIndex(target);
                    bearing = vuforia.bearing(index);
                    distance = vuforia.distance(index);
                    valid = true;
                    // TODO: Debug
                    if (true) {
                        thisTarget = target;
                        telemetry.addData("Bearing (" + thisTarget + ")", bearing);
                    }
                    break;
                }
            }
        }

        // Turn 90° left/right
        if (gamepad1.right_bumper) {
            if (gamepad1.x) {
                turnAngle(-FULL_CIRCLE / 4);
            } else if (gamepad1.b) {
                turnAngle(FULL_CIRCLE / 4);
            }
        }

        // Turn to face cardinal directions (or our best guess if we've never seen a target)
        if (gamepad1.left_bumper) {
            if (gamepad1.y) {
                turnBearing(FULL_CIRCLE / 4 * 0);
            } else if (gamepad1.b) {
                turnBearing(FULL_CIRCLE / 4 * 1);
            } else if (gamepad1.a) {
                turnBearing(FULL_CIRCLE / 4 * 2);
            } else if (gamepad1.x) {
                turnBearing(FULL_CIRCLE / 4 * 3);
            }
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
        }

        // Drive half the distance to the first visible target
        if (gamepad1.x) {
            driveForward(distance / 2);
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
                // Turning clockwise increases heading
                if (param.comparator.equals(DriveToComp.GREATER)) {
                    // TODO: Debug
                    if (true) {
                        //telemetry.log().add("Target/Bearing: " + lastTarget + "/" + lastBearing);
                        telemetry.log().add("CW current/target: " + gyro.getHeading() + "/" + param.limit1);
                    }
                    tank.setSpeed(-SPEED_TURN, MotorSide.LEFT);
                    tank.setSpeed(SPEED_TURN, MotorSide.RIGHT);
                } else {
                    // TODO: Debug
                    if (true) {
                        //telemetry.log().add("Target/Bearing: " + lastTarget + "/" + lastBearing);
                        telemetry.log().add("CCW current/target: " + gyro.getHeading() + "/" + param.limit1);
                    }
                    tank.setSpeed(SPEED_TURN, MotorSide.LEFT);
                    tank.setSpeed(-SPEED_TURN, MotorSide.RIGHT);
                }
                break;
            case GYRO_SECONDARY:
                // Do nothing
                // TODO: Debug
                if (false) {
                    telemetry.log().add("Secondary " + param.comparator + " current/target: " + gyro.getHeading() + "/" + param.limit1);
                }
                break;
            case ENCODER:
                // Always drive forward
                // TODO: Debug
                if (false) {
                    telemetry.log().add("Forward current/target: " + tank.getEncoder(ENCODER_INDEX) + "/" + param.limit1);
                }
                tank.setSpeed(-SPEED_DRIVE);
                break;
        }
    }

    @Override
    public double driveToSensor(DriveToParams param) {
        double value = 0;
        switch ((SENSOR_TYPE) param.reference) {
            case GYRO:
                // Fallthrough
            case GYRO_SECONDARY:
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

        // Normalized heading and bearing
        int heading = gyro.getHeading();
        int bearing = (heading + angle + FULL_CIRCLE) % FULL_CIRCLE;

        // Turn CCW for negative angles
        if (angle > 0) {
            param.greaterThan(bearing);
        } else {
            param.lessThan(bearing);
        }
        drive = new DriveTo(new DriveToParams[]{param});
    }

    private void turnBearing(int bearing) {
        tank.setTeleop(false);
        DriveToParams param1 = new DriveToParams(this, SENSOR_TYPE.GYRO);
        DriveToParams param2 = new DriveToParams(this, SENSOR_TYPE.GYRO_SECONDARY);

        // Normalized heading and bearing
        int heading = gyro.getHeading();
        bearing = (bearing + FULL_CIRCLE) % FULL_CIRCLE;

        // Turn the short way
        // TODO: Debug
        lastBearing = bearing;
        lastTarget = thisTarget;
        if ((bearing - heading + FULL_CIRCLE) % FULL_CIRCLE <= HALF_CIRCLE) {
            // CW
            param1.greaterThan(bearing);
            param2.lessThan((bearing + HALF_CIRCLE) % FULL_CIRCLE);
        } else {
            // CCW
            param1.lessThan(bearing);
            param2.greaterThan((bearing + HALF_CIRCLE) % FULL_CIRCLE);
        }
        drive = new DriveTo(new DriveToParams[]{param1, param2});
    }

    private void driveForward(int distance) {
        tank.setTeleop(false);
        DriveToParams param = new DriveToParams(this, SENSOR_TYPE.ENCODER);
        int ticks = (int) ((float) -distance * ENCODER_PER_MM);
        param.lessThan(ticks + tank.getEncoder(ENCODER_INDEX));
        drive = new DriveTo(new DriveToParams[]{param});
    }
}