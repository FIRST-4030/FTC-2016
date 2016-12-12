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
package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class VuforiaFTC {
    /**
     * TODO: If you downloaded this file from another team you need to get your own Vuforia key
     * See https://library.vuforia.com/articles/Solution/How-To-Create-an-App-License for instructions
     */
    // Team-specific Vuforia key
    private static final String VUFORIA_KEY = "AbgpAh3/////AAAAGTwS0imaZU6wjVVHhw7cr1iHxcyPegw1+zPNzs+oNjtZlwpyvuwb2hdTLeEEj0gPTWUgVfLbnn6BrV6pafSnN8oCEEZrbVicTGw02BT+V0IzD43++kcsLVuumaM9yAUlAaDPiuEvEx6AZxYnM05KMzlAtMtfgW8tOIvjlicxep9tPhr1Z1Z3JrDt8s8mPo3GsSRSvpoSXZfxRLi0CwGEJlTuVrP59wLhsvr3CZ5Nr7gCNznhAaiGp4LhtCPoXsIUjsQHwO2hmskW670gZGIZl7BvqVbN5mIwqOYF3ZsCUkR83pM7jSIsOMdiaLK5ZlVLG+z5AfgoPNDZo8iYiqTncIiSUL5oJuh2NIeiG+nwcPJV";

    // Short names for external constants
    public static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;
    public static final AxesReference AXES_REFERENCE = AxesReference.EXTRINSIC;
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    // Numeric constants
    private static final int FULL_CIRCLE = 360;
    private static final float MILLS_PER_SEC = 1000.0f;

    // Tracking config
    private final String CONFIG_ASSET;
    private final int CONFIG_TARGETS_NUM;
    private final VuforiaTarget[] CONFIG_TARGETS;
    private final VuforiaTarget CONFIG_PHONE;

    // Dynamic things we need to remember
    private int TRACKING_TIMEOUT = 500;
    private VuforiaTrackables TARGETS_RAW = null;
    private final List<VuforiaTrackable> TARGETS = new ArrayList<>();

    // The actual data we care about
    private long timestamp = 0;
    private final int[] location = new int[3];
    private final int[] orientation = new int[3];
    private final HashMap<String, Boolean> targetVisible = new HashMap<>();
    private final HashMap<String, Integer> targetAngle = new HashMap<>();
    private final HashMap<String, Integer> targetIndex = new HashMap<>();

    public VuforiaFTC(String targetAsset, int numTargets, VuforiaTarget[] targetConfig, VuforiaTarget phoneConfig) {
        CONFIG_TARGETS = targetConfig;
        CONFIG_PHONE = phoneConfig;
        CONFIG_ASSET = targetAsset;
        CONFIG_TARGETS_NUM = numTargets;
    }

    public void init() {
        // Init Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_DIRECTION;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Pre-processed target images from the Vuforia target manager:
         * https://developer.vuforia.com/target-manager.
         */
        TARGETS_RAW = vuforia.loadTrackablesFromAsset(CONFIG_ASSET);
        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, CONFIG_TARGETS_NUM);
        TARGETS.addAll(TARGETS_RAW);

        // Configure target names, locations, rotations and hashmaps
        for (int i = 0; i < CONFIG_TARGETS_NUM; i++) {
            initTrackable(TARGETS_RAW, i);
        }

        // Location and rotation of the image sensor plane relative to the robot
        OpenGLMatrix phoneLocation = positionRotationMatrix(CONFIG_PHONE.raw, CONFIG_PHONE.rotation, CONFIG_PHONE.axesOrder);
        for (VuforiaTrackable trackable : TARGETS) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }
    }

    public void start() {
        TARGETS_RAW.activate();
    }

    public void track() {
        for (VuforiaTrackable trackable : TARGETS) {
            // Per-target visibility (somewhat imaginary but still useful)
            targetVisible.put(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible());

            // Angle to target, if available
            OpenGLMatrix newPose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getPose();
            if (newPose != null) {
                Orientation poseOrientation = Orientation.getOrientation(newPose, AXES_REFERENCE, AxesOrder.XYZ, ANGLE_UNIT);
                targetAngle.put(trackable.getName(), (int) poseOrientation.secondAngle);
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
            }
        }
    }

    public void display(Telemetry telemetry) {

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

    /**
     * Getters
     */
    public HashMap<String, Boolean> getVisible() {
        return targetVisible;
    }

    /**
     * @param target Name of the target of interest.
     * @return True if the target was actively tracked in the last round of VuforiaFTC processing
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
     * @return The VuforiaFTC targetable index for the named target.
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
     * @param index CONFIG_TARGETS index. Syntax helper for {@link #bearing(int, int)} bearing(int, int)}
     * @return Bearing from the current location to {x,y} with respect to field north
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int bearing(int index) {
        return bearing(CONFIG_TARGETS[index].adjusted[0], CONFIG_TARGETS[index].adjusted[1]);
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
     * @param index CONFIG_TARGETS index. Syntax helper for {@link #distance(int, int)} distance(int, int)}
     * @return Distance from the current location to {x,y} with respect to field units (millimeters)
     * <p>
     * This value may be out-of-date. Most uses should include an evaluation of validity based on
     * {@link #isStale() isStale()} or {@link #getTimestamp() getTimestamp()}
     */
    public int distance(int index) {
        return distance(CONFIG_TARGETS[index].adjusted[0], CONFIG_TARGETS[index].adjusted[1]);
    }

    public void setTrackingTimeout(int timeout) {
        TRACKING_TIMEOUT = timeout;
    }

    public int getTrackingTimeout() {
        return TRACKING_TIMEOUT;
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
            RobotLog.a("Invalid VuforiaFTC trackable index: %d", index);
            return;
        }

        // Per-target hashmaps, by name
        targetIndex.put(CONFIG_TARGETS[index].name, index);
        targetVisible.put(CONFIG_TARGETS[index].name, false);
        targetAngle.put(CONFIG_TARGETS[index].name, 0);

        // Location model parameters
        VuforiaTrackable trackable = trackables.get(index);
        trackable.setName(CONFIG_TARGETS[index].name);
        OpenGLMatrix location = positionRotationMatrix(CONFIG_TARGETS[index].raw,
                CONFIG_TARGETS[index].rotation, CONFIG_TARGETS[index].axesOrder);
        trackable.setLocation(location);
    }
}
