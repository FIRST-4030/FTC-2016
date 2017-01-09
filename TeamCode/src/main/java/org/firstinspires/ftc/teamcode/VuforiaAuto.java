package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Vuforia Auto", group = "Test")
public class VuforiaAuto extends OpMode implements DriveToListener {

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
    private static final float SPEED_SHOOT = 1.0f;
    private static final float BLOCKER_UP = 0.0f;
    private static final float BLOCKER_DOWN = 0.98f;
    private static final float BOOPER_IN = 0.0f;
    private static final float BOOPER_OUT = 0.75f;

    // Autonomous routine constants
    private static final float GYRO_TIMEOUT = 5.0f;
    private static final int SHOOT_DISTANCE = 1850;
    private static final int SHOOT_SPIN = 3700;
    private static final int NUM_SHOTS = 2;
    private static final float SHOT_DELAY = 1.0f;
    private static final int BALL_DISTANCE = 1100;
    private static final int BLIND_TURN = 30;
    private static final int DESTINATION_OFFSET = 100;
    private static final float BEACON_DELAY = 1.0f;

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

    // Devices and subsystems
    private VuforiaFTC vuforia;
    private TankDrive tank;
    private Gyro gyro;
    private DriveTo drive;
    private DcMotor shooter;
    private Servo blocker;
    private Servo booperLeft;
    private Servo booperRight;

    // Dynamic things we need to remember
    private double headingSyncExpires = 0;
    private AUTO_STATE state = AUTO_STATE.INIT;
    private int shots = 0;
    private double timer = 0.0;
    private int target = -1;
    private AllianceColor.Color beacon = null;

    // Sensor reference types for our DriveTo callbacks
    enum SENSOR_TYPE {
        GYRO, DRIVE_ENCODER, SHOOT_ENCODER
    }

    // Enum for the main state machine
    enum AUTO_STATE {
        INIT,
        DRIVE_TO_SHOOT,
        SHOOT,
        SHOOT_WAIT,
        DRIVE_TO_BALL,
        BLIND_TURN,
        FIND_TARGET,
        TURN_TO_DEST,
        DRIVE_TO_DEST,
        TURN_TO_TARGET,
        ALIGN_AT_TARGET,
        CHECK_COLOR,
        PRESS_BEACON,
        BEACON_WAIT,
        BACK_AWAY,
        DONE
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
            telemetry.log().add("ERROR: Unable to initalize wheels");
        }

        // Shooter motor
        try {
            shooter = hardwareMap.dcMotor.get("shooter");
        } catch (Exception e) {
            telemetry.log().add("ERROR: Unable to initalize shooter");
            shooter = null;
        }

        // Blocker
        try {
            blocker = hardwareMap.servo.get("blocker");
            blocker.setPosition(BLOCKER_DOWN);
        } catch (Exception e) {
            telemetry.log().add("ERROR: Unable to initalize blocker");
            blocker = null;
        }

        // Boopers
        try {
            booperLeft = hardwareMap.servo.get("booper-left");
            booperRight = hardwareMap.servo.get("booper-right");
            booperRight.setDirection(Servo.Direction.REVERSE);
            booperLeft.setPosition(BOOPER_IN);
            booperRight.setPosition(BOOPER_IN);
        } catch (Exception e) {
            telemetry.log().add("ERROR: Unable to initalize boopers");
            booperLeft = null;
            booperRight = null;
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

        // Steady...
        tank.stop();
        state = AUTO_STATE.INIT;
        timer = time + GYRO_TIMEOUT;
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
        telemetry.addData("State", state);
        vuforia.display(telemetry);
        telemetry.addData("Encoder", tank.getEncoder(ENCODER_INDEX));
        if (!gyro.isReady()) {
            telemetry.addData("Gyro", "Calibrating (DO NOT DRIVE): %d" + (int) time);
        } else {
            telemetry.addData("Gyro Abs/Rel", gyro.getHeading() + "°/" + gyro.getHeadingRaw() + "°");
        }
        telemetry.update();

        /*
         * Cut the loop short when we are auto-driving
         * This keeps us out of the state machine until the last auto-drive command is complete
         */
        if (drive != null) {
            return;
        }

        // Update our location and target info
        vuforia.track();

        // Update the gyro offset if we have a fix
        if (!vuforia.isStale() && headingSyncExpires < time) {
            headingSyncExpires = time + GYRO_MIN_UPDATE_INTERVAL;
            gyro.setHeading(vuforia.getHeading());
        }

        // Main state machine
        DriveToParams param;
        switch (state) {
            case INIT:
                if (gyro.isReady()) {
                    state = AUTO_STATE.DRIVE_TO_SHOOT;
                } else if (timer < time) {
                    gyro.disable();
                    state = AUTO_STATE.DRIVE_TO_SHOOT;
                }
                break;
            case DRIVE_TO_SHOOT:
                driveForward(SHOOT_DISTANCE);
                state = AUTO_STATE.SHOOT;
                break;
            case SHOOT:
                if (blocker != null) {
                    blocker.setPosition(BLOCKER_UP);
                }
                param = new DriveToParams(this, SENSOR_TYPE.SHOOT_ENCODER);
                param.greaterThan(SHOOT_SPIN);
                drive = new DriveTo(new DriveToParams[]{param});
                timer = time + SHOT_DELAY;
                shots++;
                break;
            case SHOOT_WAIT:
                if (blocker != null) {
                    blocker.setPosition(BLOCKER_DOWN);
                }
                if (shots >= NUM_SHOTS) {
                    state = AUTO_STATE.DRIVE_TO_BALL;
                } else if (timer < time) {
                    state = AUTO_STATE.SHOOT;
                }
                break;
            case DRIVE_TO_BALL:
                driveForward(BALL_DISTANCE);
                state = AUTO_STATE.TURN_TO_TARGET;
                break;
            case BLIND_TURN:
                // Bail if we have no gyro
                if (!gyro.isReady()) {
                    state = AUTO_STATE.DONE;
                }
                turnAngle(BLIND_TURN);
                state = AUTO_STATE.FIND_TARGET;
                break;
            case FIND_TARGET:
                // TODO: Spin or something

                // Select a target when we have a vision fix
                if (!vuforia.isStale()) {
                    target = closestTarget();
                    state = AUTO_STATE.TURN_TO_DEST;
                }
                break;
            case TURN_TO_DEST:
                if (gyro.isReady() && target >= 0) {
                    turnBearing(vuforia.bearing(destinationXY(target)));
                    state = AUTO_STATE.DRIVE_TO_DEST;
                }
                break;
            case DRIVE_TO_DEST:
                if (target >= 0) {
                    driveForward(vuforia.distance(destinationXY(target)));
                }
                state = AUTO_STATE.TURN_TO_TARGET;
                break;
            case TURN_TO_TARGET:
                if (gyro.isReady() && target >= 0) {
                    turnBearing(vuforia.bearing(target));
                    state = AUTO_STATE.ALIGN_AT_TARGET;
                }
                break;
            case ALIGN_AT_TARGET:
                // TODO: Some sort of turn and approach operation
                if (vuforia.getVisible(CONFIG[target].name)) {
                    state = AUTO_STATE.CHECK_COLOR;
                }
                break;
            case CHECK_COLOR:
                // TODO: Check the color sensor for beacon configuration
                beacon = AllianceColor.Color.BLUE;
                state = AUTO_STATE.PRESS_BEACON;
                break;
            case PRESS_BEACON:
                // TODO: This depends on where the color sensor is mounted
                if (beacon.equals(AllianceColor.Color.RED)) {
                    if (booperLeft != null) {
                        booperLeft.setPosition(BOOPER_OUT);
                    }
                } else {
                    if (booperRight != null) {
                        booperRight.setPosition(BOOPER_OUT);
                    }
                }
                timer = time + BEACON_DELAY;
                state = AUTO_STATE.BEACON_WAIT;
                break;
            case BEACON_WAIT:
                if (timer < time) {
                    state = AUTO_STATE.BACK_AWAY;
                }
                break;
            case BACK_AWAY:
                if (booperLeft != null && booperRight != null) {
                    booperLeft.setPosition(BOOPER_IN);
                    booperRight.setPosition(BOOPER_IN);
                }
                driveForward(-DESTINATION_OFFSET);
                state = AUTO_STATE.DONE;
                break;
            case DONE:
                // Nothing
                break;
        }
    }

    @Override
    public void driveToStop(DriveToParams param) {
        switch ((SENSOR_TYPE) param.reference) {
            case GYRO:
                // Fall through
            case DRIVE_ENCODER:
                tank.stop();
                break;
            case SHOOT_ENCODER:
                if (shooter != null) {
                    shooter.setPower(0);
                }
                break;
        }
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
            case DRIVE_ENCODER:
                tank.setSpeed(-SPEED_DRIVE);
                break;
            case SHOOT_ENCODER:
                if (shooter != null) {
                    shooter.setPower(SPEED_SHOOT);
                }
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
            case DRIVE_ENCODER:
                value = tank.getEncoder(ENCODER_INDEX);
                break;
            case SHOOT_ENCODER:
                if (shooter != null) {
                    value = shooter.getCurrentPosition();
                }
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
        DriveToParams param = new DriveToParams(this, SENSOR_TYPE.DRIVE_ENCODER);
        int ticks = (int) ((float) -distance * ENCODER_PER_MM);
        param.lessThan(ticks + tank.getEncoder(ENCODER_INDEX) - OVERRUN_ENCODER);
        drive = new DriveTo(new DriveToParams[]{param});
    }

    private int closestTarget() {
        int index = -1;
        int min = Integer.MAX_VALUE;
        for (int i = 0; i < CONFIG.length; i++) {
            int distance = vuforia.distance(CONFIG[i].adjusted[0], CONFIG[i].adjusted[1]);
            if (distance < min) {
                min = distance;
                index = i;
            }
        }
        return index;
    }

    private int[] destinationXY(int index) {
        int[] destination = new int[]{CONFIG[index].adjusted[0], CONFIG[index].adjusted[1]};
        // TODO: This is imaginary and must be calibrated before use
        if (CONFIG[index].color.equals(AllianceColor.Color.RED)) {
            destination[0] -= DESTINATION_OFFSET;
        } else {
            destination[1] -= DESTINATION_OFFSET;
        }
        return destination;
    }
}