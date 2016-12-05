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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends TankOpMode {

    public static final String TAG = "Vuforia Test";
    private static final boolean DISPLAY_STD = true;
    private static final boolean DISPLAY_TARGET = false;
    private static final boolean DISPLAY_LOCATION = false;

    // Short names for external constants
    public static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;
    public static final AxesReference AXES_REFERENCE = AxesReference.EXTRINSIC;
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;
    public static final int FULL_CIRCLE = 360;

    // Tracking targets
    public static final String TARGETS_FILE = "FTC_2016-17";
    public static final int TARGETS_NUM = 4;
    public static final float[] TARGETS_ROTATION_RED = {90, 90, 0};
    public static final float[] TARGETS_ROTATION_BLUE = {90, 0, 0};

    // Field, camera and robot constants
    public static final int TRACKING_TIMEOUT = 500;
    public static final float MILLS_PER_SEC = 1000.0f;
    public static final float MM_PER_INCH = 25.4f;
    public static final float BOT_WIDTH = 18 * MM_PER_INCH;
    public static final float FIELD_WIDTH = (12 * 12 - 2) * MM_PER_INCH;

    // Team-specific Vuforia key
    // TODO: If you downloaded this file from another team you need to get your own Vuforia key
    private static final String VUFORIA_KEY = "AbgpAh3/////AAAAGTwS0imaZU6wjVVHhw7cr1iHxcyPegw1+zPNzs+oNjtZlwpyvuwb2hdTLeEEj0gPTWUgVfLbnn6BrV6pafSnN8oCEEZrbVicTGw02BT+V0IzD43++kcsLVuumaM9yAUlAaDPiuEvEx6AZxYnM05KMzlAtMtfgW8tOIvjlicxep9tPhr1Z1Z3JrDt8s8mPo3GsSRSvpoSXZfxRLi0CwGEJlTuVrP59wLhsvr3CZ5Nr7gCNznhAaiGp4LhtCPoXsIUjsQHwO2hmskW670gZGIZl7BvqVbN5mIwqOYF3ZsCUkR83pM7jSIsOMdiaLK5ZlVLG+z5AfgoPNDZo8iYiqTncIiSUL5oJuh2NIeiG+nwcPJV";

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

        // Name and locate the targets
        // TODO: These locations are imaginary. We need to find the real ones before navigation.
        initTrackable(TARGETS_RAW, 0, "Wheels", new float[]{-FIELD_WIDTH / 2, 0, 0}, TARGETS_ROTATION_RED);
        initTrackable(TARGETS_RAW, 1, "Tools", new float[]{-FIELD_WIDTH / 2, FIELD_WIDTH / 4, 0}, TARGETS_ROTATION_RED);
        initTrackable(TARGETS_RAW, 2, "LEGO", new float[]{0, FIELD_WIDTH / 2, 0}, TARGETS_ROTATION_BLUE);
        initTrackable(TARGETS_RAW, 3, "Gears", new float[]{FIELD_WIDTH / 4, FIELD_WIDTH / 2, 0}, TARGETS_ROTATION_BLUE);
        TARGETS.addAll(TARGETS_RAW);

        // Position and rotation of the image sensor plane relative to the robot
        // TODO: This location and pose may also be imaginary, but should at least be close.
        OpenGLMatrix phoneLocation = positionRotationMatrix(new float[]{BOT_WIDTH / 2, 0, 0}, new float[]{-90, 0, 0}, AxesOrder.YZY);
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
        for (VuforiaTrackable trackable : TARGETS) {
            targetVisible.put(trackable.getName(), false);
        }
        for (VuforiaTrackable trackable : TARGETS) {
            targetAngle.put(trackable.getName(), 0);
        }
    }

    @Override
    public void loop() {
        // Inherit TankDrive functionality (if enabled)
        if (!DISABLE_TELEOP) {
            super.loop();
        }

        // Update our location and pose
        doTracking();

        // TODO: Presumably automated driving of some sort

        // Feedback to the DS
        if (DISPLAY_STD) {
            telemetry.addData("Valid", isStale() ? "No" : "Yes");

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

            for (String target : targetVisible.keySet()) {
                if (getVisible(target)) {
                    telemetry.addData(target + " ∠", getTargetAngle(target) + "°");
                }
            }

            telemetry.addData("X/Y Heading", getX() + "/" + getY() + " " + getHeading() + "°");
            if (getTimestamp() > 0) {
                telemetry.addData("Age", "%.2f", (System.currentTimeMillis() - getTimestamp()) / MILLS_PER_SEC);
            }
        }

        // Loop invariants
        telemetry.update();
    }

    // TODO: Consider spawning a thread for this loop (if allowed by the framework)
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

    /**
     * Getters
     */
    public HashMap<String, Boolean> getVisible() {
        return targetVisible;
    }

    /**
     *
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
     *
     * @param target Name of the target of interest. Valid targets will also be visible per
     * {@link #getVisible(String)} getVisible(target)}
     * @return The angle to the target's plane relative to the plane of the phone's image sensor
     * (i.e. 0° is dead-on, negative sign denotes right-of-center)
     */
    public int getTargetAngle(String target) {
        return targetAngle.get(target);
    }

    /**
     *
     * @return System.currentTimeMillis() as reported at the time of the last location update
     */
    public long getTimestamp() {
        return timestamp;
    }

    /**
     *
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
     *
     * @return The X component of the robot's last known location relative to the field center.
     * Negative values denote blue alliance side of field.
     *
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int getX() {
        return location[0];
    }

    /**
     *
     * @return The Y component of the robot's last known location relative to the field center.
     * Negative sign denotes audiance side of field.
     *
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int getY() {
        return location[1];
    }

    /**
     *
     * @return The robot's last known heading relative to the field.
     *
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
     * Helpers
     */

    // It's like a macro, but for Java
    private OpenGLMatrix positionRotationMatrix(float[] position, float[] rotation, AxesOrder order) {
        return OpenGLMatrix
                .translation(position[0], position[1], position[2])
                .multiplied(Orientation.getRotationMatrix(
                        AXES_REFERENCE, order, ANGLE_UNIT,
                        rotation[0], rotation[1], rotation[2]));
    }

    // More Java blasphemy
    private void initTrackable(VuforiaTrackables trackables, int index, String name, float[] position, float[] rotation) {
        if (index >= trackables.size() || index < 0) {
            RobotLog.a("Invalid Vuforia trackable index (%d) for target: %s", index, name);
            return;
        }

        VuforiaTrackable trackable = trackables.get(index);
        trackable.setName(name);
        OpenGLMatrix location = positionRotationMatrix(position, rotation, AxesOrder.XZX);
        trackable.setLocation(location);
    }
}
