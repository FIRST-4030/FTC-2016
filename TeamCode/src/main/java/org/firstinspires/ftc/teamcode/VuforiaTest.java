/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.classes.TankOpMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

class Target {
    public final String name;
    public final float[] raw;
    public final float[] offset;
    public final float[] rotation;
    public final AxesOrder axesOrder;
    public final int[] adjusted = new int[3];

    public Target(String name, float[] location, float[] offset, float[] rotation) {
        this.name = name;
        this.raw = location;
        this.offset = offset;
        this.rotation = rotation;
        this.axesOrder = AxesOrder.XZX;

        for (int i = 0; i < 3; i++) {
            this.adjusted[i] = (int) (location[i] + offset[i]);
        }
    }
}

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends TankOpMode {

    public static final String TAG = "Vuforia Test";

    // Debug flags
    private static final boolean DISPLAY_STD = true;
    private static final boolean DISPLAY_TARGET = false;
    private static final boolean DISPLAY_LOCATION = false;
    private static final boolean DISPLAY_BEARING = false;

    // Team-specific Vuforia key
    // TODO: If you downloaded this file from another team you need to get your own Vuforia key
    private static final String VUFORIA_KEY = "AbgpAh3/////AAAAGTwS0imaZU6wjVVHhw7cr1iHxcyPegw1+zPNzs+oNjtZlwpyvuwb2hdTLeEEj0gPTWUgVfLbnn6BrV6pafSnN8oCEEZrbVicTGw02BT+V0IzD43++kcsLVuumaM9yAUlAaDPiuEvEx6AZxYnM05KMzlAtMtfgW8tOIvjlicxep9tPhr1Z1Z3JrDt8s8mPo3GsSRSvpoSXZfxRLi0CwGEJlTuVrP59wLhsvr3CZ5Nr7gCNznhAaiGp4LhtCPoXsIUjsQHwO2hmskW670gZGIZl7BvqVbN5mIwqOYF3ZsCUkR83pM7jSIsOMdiaLK5ZlVLG+z5AfgoPNDZo8iYiqTncIiSUL5oJuh2NIeiG+nwcPJV";

    // Short names for external constants
    public static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;
    public static final AxesReference AXES_REFERENCE = AxesReference.EXTRINSIC;
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    public static final int FULL_CIRCLE = 360;

    // Field, camera and robot constants
    public static final int TRACKING_TIMEOUT = 500;
    public static final float MILLS_PER_SEC = 1000.0f;
    public static final float MM_PER_INCH = 25.4f;
    public static final float BOT_WIDTH = 18 * MM_PER_INCH;
    public static final float FIELD_WIDTH = (12 * 12 - 2) * MM_PER_INCH;

    // Phone constants
    // TODO: This location and rotation is imaginary, but should at least be close.
    public static final float[] PHONE_LOCATION = {BOT_WIDTH / 2, 0, 0};
    public static final float[] PHONE_ROTATION = {-90, 0, 0};
    private static final AxesOrder PHONE_AXES_ORDER = AxesOrder.YZY;

    // Tracking targets
    public static final String TARGETS_FILE = "FTC_2016-17";
    public static final int TARGETS_NUM = 4;
    // TODO: These locations are imaginary. We need to find the real ones before navigation.
    private static final float[] TARGETS_ROTATION_RED = {270, 180, 0};
    private static final float[] TARGETS_ROTATION_BLUE = {270, 270, 0};
    private static final float[] TARGETS_OFFSET_RED = {250, 0, 0};
    private static final float[] TARGETS_OFFSET_BLUE = {0, -250, 0};
    private static final float TARGETS_Y_BLUE = FIELD_WIDTH / 2;
    private static final float TARGETS_X_RED = FIELD_WIDTH / 2;
    private static final Target[] CONFIG = {new Target(
            "Wheels",
            new float[]{100, TARGETS_Y_BLUE, 0},
            TARGETS_OFFSET_BLUE,
            TARGETS_ROTATION_BLUE
    ), new Target(
            "Tools",
            new float[]{TARGETS_X_RED, 600, 0},
            TARGETS_OFFSET_RED,
            TARGETS_ROTATION_RED
    ), new Target(
            "LEGO",
            new float[]{-600, TARGETS_Y_BLUE, 0},
            TARGETS_OFFSET_BLUE,
            TARGETS_ROTATION_BLUE
    ), new Target(
            "Gears",
            new float[]{TARGETS_X_RED, -100, 0},
            TARGETS_OFFSET_RED,
            TARGETS_ROTATION_RED
    )};

    // Driving constants
    private final float ENCODER_PER_MM = 1.0f;
    private final int DRIVE_MAX_TURN = 15;

    // Dynamic things we need to remember
    private VuforiaTrackables TARGETS_RAW = null;
    private final List<VuforiaTrackable> TARGETS = new ArrayList<>();
    private boolean DISABLE_TELEOP = false;

    // The actual data we care about
    private long timestamp = 0;
    private final int[] location = new int[3];
    private final int[] orientation = new int[3];
    private final HashMap<String, Boolean> targetVisible = new HashMap<>();
    private final HashMap<String, Integer> targetAngle = new HashMap<>();
    private final HashMap<String, Integer> targetIndex = new HashMap<>();

    // Tank Drive constructor
    @SuppressWarnings("unused")
    public VuforiaTest() {
        super("front left", "front right", "back left", "back right");
    }

    @Override
    public void init() {

        // Placate drivers; sometimes Vuforia is slow to init
        telemetry.addData(">", "Initalizing...");
        telemetry.update();

        // Init Tank Drive
        // Try/Catch to keep running even if we can't talk to the hardware
        try {
            super.init();
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            DISABLE_TELEOP = true;
            telemetry.log().add("ERROR: Unable to initalize motors");
            telemetry.update();
        }

        // Init Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_DIRECTION;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Pre-processed target images from the Vuforia target manager:
         * https://developer.vuforia.com/target-manager.
         */
        TARGETS_RAW = vuforia.loadTrackablesFromAsset(TARGETS_FILE);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, TARGETS_NUM);
        TARGETS.addAll(TARGETS_RAW);

        // Configure target names, locations, rotations and hashmaps
        for (int i = 0; i < TARGETS_NUM; i++) {
            initTrackable(TARGETS_RAW, i);
        }

        // Location and rotation of the image sensor plane relative to the robot
        OpenGLMatrix phoneLocation = positionRotationMatrix(PHONE_LOCATION, PHONE_ROTATION, PHONE_AXES_ORDER);
        for (VuforiaTrackable trackable : TARGETS) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }

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
        TARGETS_RAW.activate();
    }

    @Override
    public void loop() {
        // Inherit TankDrive functionality (if enabled)
        if (!DISABLE_TELEOP) {
            super.loop();
        }

        // Update our location and pose
        doTracking();

        // Standard (driver) feedback to the DS
        if (DISPLAY_STD) {
            doStandardDisplay();
        }

        // Bearing debug
        if (DISPLAY_BEARING && !isStale()) {
            doBearingDebug();
        }

        // Turn to face the first visible target and drive half the distance toward it
        if (gamepad1.y && !isStale()) {
            boolean valid = false;
            int bearing = 0;
            int distance = 0;

            // Get data for the first visible target
            for (String target : targetVisible.keySet()) {
                if (getVisible(target)) {
                    int index = getTargetIndex(target);
                    bearing = bearing(index);
                    distance = distance(index);
                    valid = true;
                    break;
                }
            }

            // If we're tracking a target
            if (valid) {
                int ticks = (int) (distance * ENCODER_PER_MM) / 2;
                int angle = getHeading() - bearing;

                // Drive only if the turn is likely to keep us at a tracking angle
                if (Math.abs(angle) < DRIVE_MAX_TURN) {
                    // TODO: Something that makes us turn <angle> degrees and then drive <ticks> encoder ticks
                }
            }

            // Loop invariants
            telemetry.update();
        }
    }

    private void doTracking() {
        for (VuforiaTrackable trackable : TARGETS) {
            // Per-target visibility (somewhat imaginary but still useful)
            targetVisible.put(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible());

            // Angle to target, if available
            OpenGLMatrix newPose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
            if (newPose != null) {
                Orientation poseOrientation = Orientation.getOrientation(newPose, AXES_REFERENCE, AxesOrder.XYZ, ANGLE_UNIT);
                targetAngle.put(trackable.getName(), (int) poseOrientation.secondAngle);

                if (DISPLAY_TARGET) {
                    doTargetDebug(trackable, newPose, poseOrientation);
                }
            }

            /**
             * Update the location and orientation track
             *
             * We poll for each trackable so this happens in the loop, but the overall tracking
             * is aggregated among all targets with a defined pose and location. The current
             * field of view will dictate the quality of the track and if one or more targets
             * are present they will be the primary basis for tracking but tracking persists
             * even when the view does not include a target, and is self-consistent when the
             * view includes multiple targets
             */
            OpenGLMatrix newLocation = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
            // TODO: Find out if we need this or can get the pose from the location matrix
            if (newLocation != null) {
                // Extract our location from the matrix
                for (int i = 0; i < location.length; i++) {
                    location[i] = (int) newLocation.get(i, 3);
                }

                // Calculate the orientation of our view
                Orientation newOrientation = Orientation.getOrientation(newLocation, AXES_REFERENCE, AxesOrder.XYZ, ANGLE_UNIT);
                orientation[0] = (int) newOrientation.firstAngle;
                orientation[1] = (int) newOrientation.secondAngle;
                orientation[2] = (int) newOrientation.thirdAngle;

                // Timestamp the update
                timestamp = System.currentTimeMillis();

                // Debug telemetry, if enabled
                if (DISPLAY_LOCATION) {
                    telemetry.setAutoClear(false);
                    telemetry.clearAll();

                    for (int i = 0; i < newLocation.numRows(); i++) {
                        for (int j = 0; j < newLocation.numCols(); j++) {
                            telemetry.addData("Loc[" + i + "][" + j + "]", newLocation.get(i, j));
                        }
                    }
                    for (int i = 0; i < orientation.length; i++) {
                        telemetry.addData("Rot[" + i + "]", orientation[i]);
                    }
                    telemetry.addData("Heading", getHeading());
                    telemetry.addData("Location", newLocation.formatAsTransform());
                }
            }
        }
    }

    private void doStandardDisplay() {
        // Is the location track valid?
        telemetry.addData("Valid", isStale() ? "No" : "Yes");

        // List of visible targets (if any)
        String visibleStr = "";
        for (String target : targetVisible.keySet()) {
            if (getVisible(target)) {
                if (!visibleStr.isEmpty()) {
                    visibleStr += ", ";
                }
                visibleStr += target;
            }
        }
        if (visibleStr.isEmpty()) {
            visibleStr = "<None>";
        }
        telemetry.addData("Visible", visibleStr);

        // Angle to each visible target (if any)
        for (String target : targetVisible.keySet()) {
            if (getVisible(target)) {
                telemetry.addData(target + " ∠", getTargetAngle(target) + "°");
            }
        }

        // Raw data from the last location and orientation fix
        telemetry.addData("X/Y Heading", getX() + "/" + getY() + " " + getHeading() + "°");
        if (getTimestamp() > 0) {
            telemetry.addData("Age", "%.2f", (System.currentTimeMillis() - getTimestamp()) / MILLS_PER_SEC);
        }
    }


    private void doTargetDebug(VuforiaTrackable trackable, OpenGLMatrix newPose, Orientation poseOrientation) {
        telemetry.setAutoClear(false);
        telemetry.clearAll();

        telemetry.addData("Pose", trackable.getName());
        for (int i = 0; i < newPose.numRows(); i++) {
            for (int j = 0; j < newPose.numCols(); j++) {
                telemetry.addData("Pose[" + i + "][" + j + "]", newPose.get(i, j));
            }
        }

        telemetry.addData("PoseRot0", poseOrientation.firstAngle);
        telemetry.addData("PoseRot1", poseOrientation.secondAngle);
        telemetry.addData("PoseRot2", poseOrientation.thirdAngle);
    }

    // Bearing to various points on the field with respect to field north (if location is valid)
    private void doBearingDebug() {
        telemetry.addData("Field Center", distance(0, 0) + "mm @ " + bearing(0, 0) + "°");
        for (int i = 0; i < TARGETS_NUM; i++) {
            telemetry.addData(CONFIG[i].name,
                    distance(i) + "mm @ " + bearing(i) + "° (" +
                            CONFIG[i].adjusted[0] + ", " +
                            CONFIG[i].adjusted[1] + ")");
        }
    }

    /**
     * Getters
     */
    public HashMap<String, Boolean> getVisible() {
        return targetVisible;
    }

    /**
     * @param target Name of the target of interest.
     * @return True if the target was actively tracked in the last round of Vuforia processing
     */
    public boolean getVisible(String target) {
        return targetVisible.get(target);
    }

    public HashMap<String, Integer> getTargetAngle() {
        return targetAngle;
    }

    /**
     * @param target Name of the target of interest. Valid targets will also be visible per
     *               {@link #getVisible(String)} getVisible(target)}
     * @return The angle to the target's plane relative to the plane of the phone's image sensor
     * (i.e. 0° is dead-on, negative sign denotes right-of-center)
     */
    public int getTargetAngle(String target) {
        return targetAngle.get(target);
    }

    /**
     * @param target Name of the target of interest.
     * @return The Vuforia targetable index for the named target.
     */
    public int getTargetIndex(String target) {
        return targetIndex.get(target);
    }

    /**
     * @return System.currentTimeMillis() as reported at the time of the last location update
     */
    public long getTimestamp() {
        return timestamp;
    }

    /**
     * @return True when the last location update was more than TRACKING_TIMOEUT milliseconds ago
     */
    public boolean isStale() {
        return (timestamp + TRACKING_TIMEOUT < System.currentTimeMillis());
    }

    public int[] getLocation() {
        return location;
    }

    public int[] getOrientation() {
        return orientation;
    }

    /**
     * @return The X component of the robot's last known location relative to the field center.
     * Negative values denote blue alliance side of field.
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int getX() {
        return location[0];
    }

    /**
     * @return The Y component of the robot's last known location relative to the field center.
     * Negative sign denotes audiance side of field.
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int getY() {
        return location[1];
    }

    /**
     * @return The robot's last known heading relative to the field.
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int getHeading() {
        int heading = orientation[2];
        if (orientation[0] < 0) {
            heading -= FULL_CIRCLE / 2;
        }
        if (heading < 0) {
            heading += FULL_CIRCLE;
        }
        return heading;
    }

    /**
     * @param x X component of destination in the field plane
     * @param y Y component of destination in the field plane
     * @return Bearing from the current location to {x,y} with respect to field north
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int bearing(int x, int y) {
        return (int) bearing(new double[]{getX(), getY()}, new double[]{x, y});
    }

    /**
     * @param index CONFIG index. Syntax helper for {@link #bearing(int, int)} bearing(int, int)}
     * @return Bearing from the current location to {x,y} with respect to field north
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    private int bearing(int index) {
        return bearing(CONFIG[index].adjusted[0], CONFIG[index].adjusted[1]);
    }

    /**
     * @param x X component of destination in the field plane
     * @param y Y component of destination in the field plane
     * @return Distance from the current location to {x,y} with respect to field units (millimeters)
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int distance(int x, int y) {
        return (int) distance(new double[]{getX(), getY()}, new double[]{x, y});
    }

    /**
     * @param index CONFIG index. Syntax helper for {@link #distance(int, int)} distance(int, int)}
     * @return Distance from the current location to {x,y} with respect to field units (millimeters)
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    private int distance(int index) {
        return distance(CONFIG[index].adjusted[0], CONFIG[index].adjusted[1]);
    }

    /**
     * Helpers
     */

    // Bearing from x1,y1 to x2,y2 in degrees
    // Motion from south to north is correlated with increasing Y components in field locations
    private double bearing(double[] src, double[] dest) {
        double bearing = Math.atan2(dest[1] - src[1], dest[0] - src[0]);
        if (bearing < 0) {
            bearing += Math.PI * 2.0;
        }
        return Math.toDegrees(bearing);
    }

    // Distance from x1,y1 to x2,y2 in field location units (millimeters)
    private double distance(double[] src, double[] dest) {
        return Math.hypot((dest[1] - src[1]), (dest[0] - src[0]));
    }

    // It's like a macro, but for Java
    private OpenGLMatrix positionRotationMatrix(float[] position, float[] rotation, AxesOrder order) {
        return OpenGLMatrix
                .translation(position[0], position[1], position[2])
                .multiplied(Orientation.getRotationMatrix(
                        AXES_REFERENCE, order, ANGLE_UNIT,
                        rotation[0], rotation[1], rotation[2]));
    }

    // More Java blasphemy
    private void initTrackable(VuforiaTrackables trackables, int index) {
        if (index >= trackables.size() || index < 0) {
            RobotLog.a("Invalid Vuforia trackable index: %d", index);
            return;
        }

        // Per-target hashmaps, by name
        targetIndex.put(CONFIG[index].name, index);
        targetVisible.put(CONFIG[index].name, false);
        targetAngle.put(CONFIG[index].name, 0);

        // Location model parameters
        VuforiaTrackable trackable = trackables.get(index);
        trackable.setName(CONFIG[index].name);
        OpenGLMatrix location = positionRotationMatrix(CONFIG[index].raw,
                CONFIG[index].rotation, CONFIG[index].axesOrder);
        trackable.setLocation(location);
    }
}
