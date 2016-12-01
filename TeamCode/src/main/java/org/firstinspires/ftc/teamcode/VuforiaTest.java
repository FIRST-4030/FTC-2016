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
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends OpMode {

    @SuppressWarnings("WeakerAccess")
    public static final String TAG = "Vuforia Test";
    public static final boolean DEBUG = false;

    // Short names for external constants
    public static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.BACK;
    public static final AxesReference AXES_REFERENCE = AxesReference.EXTRINSIC;
    public static final AngleUnit ANGLE_UNIT = AngleUnit.DEGREES;

    // Tracking targets
    public static final String TARGETS_FILE = "FTC_2016-17";
    public static final int TARGETS_NUM = 4;
    public static final float[] TARGETS_ROTATION_RED = {90, 90, 0};
    public static final float[] TARGETS_ROTATION_BLUE = {90, 0, 0};

    // Field, camera and robot constants
    public static final int TRACKING_TIMEOUT = 1000;
    public static final float MM_PER_INCH = 25.4f;
    public static final float BOT_WIDTH = 18 * MM_PER_INCH;
    public static final float FIELD_WIDTH = (12 * 12 - 2) * MM_PER_INCH;

    // Team-specific Vuforia key
    // TODO: If you downloaded this file from another team you need to get your own Vuforia key
    private static final String VUFORIA_KEY = "AbgpAh3/////AAAAGTwS0imaZU6wjVVHhw7cr1iHxcyPegw1+zPNzs+oNjtZlwpyvuwb2hdTLeEEj0gPTWUgVfLbnn6BrV6pafSnN8oCEEZrbVicTGw02BT+V0IzD43++kcsLVuumaM9yAUlAaDPiuEvEx6AZxYnM05KMzlAtMtfgW8tOIvjlicxep9tPhr1Z1Z3JrDt8s8mPo3GsSRSvpoSXZfxRLi0CwGEJlTuVrP59wLhsvr3CZ5Nr7gCNznhAaiGp4LhtCPoXsIUjsQHwO2hmskW670gZGIZl7BvqVbN5mIwqOYF3ZsCUkR83pM7jSIsOMdiaLK5ZlVLG+z5AfgoPNDZo8iYiqTncIiSUL5oJuh2NIeiG+nwcPJV";

    // Dynamic things we need to remember
    private List<VuforiaTrackable> TARGETS = new ArrayList<>();

    // The actual data we care about
    private long timestamp = 0;
    private int[] location = new int[3];
    private int[] orientation = new int[3];
    private HashMap<String, Boolean> visible = new HashMap<>();

    @Override
    public void init() {
        // Placate drivers; sometimes Vuforia is slow to init
        telemetry.addData(">", "Initalizing...");
        telemetry.update();

        // Init Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_DIRECTION;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Pre-processed target images from the Vuforia target manager:
         * https://developer.vuforia.com/target-manager.
         */
        VuforiaTrackables targets = vuforia.loadTrackablesFromAsset(TARGETS_FILE);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, TARGETS_NUM);

        // Name and locate the targets
        // TODO: These locations are imaginary. We need to find the real ones before navigation.
        initTrackable(targets, 0, "Wheels", new float[]{-FIELD_WIDTH / 2, 0, 0}, TARGETS_ROTATION_RED);
        initTrackable(targets, 1, "Tools", new float[]{-FIELD_WIDTH / 2, FIELD_WIDTH / 4, 0}, TARGETS_ROTATION_RED);
        initTrackable(targets, 3, "LEGO", new float[]{0, FIELD_WIDTH / 2, 0}, TARGETS_ROTATION_BLUE);
        initTrackable(targets, 4, "Gears", new float[]{FIELD_WIDTH / 4, FIELD_WIDTH / 2, 0}, TARGETS_ROTATION_BLUE);
        TARGETS.addAll(targets);

        // Position and rotation of the image sensor plane relative to the robot
        // TODO: This location and pose may also be imaginary, but should at least be close.
        OpenGLMatrix phoneLocation = positionRotationMatrix(new float[]{BOT_WIDTH / 2, 0, 0}, new float[]{-90, 0, 0}, AxesOrder.YZY);
        for (VuforiaTrackable trackable : TARGETS) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocation, parameters.cameraDirection);
        }

        // Start Vuforia tracking
        targets.activate();
        for (VuforiaTrackable trackable : TARGETS) {
            visible.put(trackable.getName(), false);
        }

        // Wait for the game to begin
        telemetry.addData(">", "Ready");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.clearAll();
    }

    @Override
    public void loop() {
        // Update our location and pose
        doTracking();

        // Feedback to the DS
        telemetry.addData("X/Y/Heading", getX() + "/" + getY() + "/" + getHeading());
        telemetry.addData("Stale", isStale() ? "Yes" : "No");
        telemetry.addData("Visible", Arrays.toString(visible.keySet().toArray()));
        telemetry.addData("Update", getTimestamp());

        // TODO: Presumably driving or something

        // Loop invariants
        telemetry.update();
    }

    // TODO: Consider spawning a thread for this loop (if allowed by the framework)
    private void doTracking() {
        for (VuforiaTrackable trackable : TARGETS) {
            visible.put(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible());

            /**
             * Update the location and pose track
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
                // TODO: This is an untested guess
                Orientation newOrientation = Orientation.getOrientation(newLocation, AXES_REFERENCE, AxesOrder.XYZ, ANGLE_UNIT);
                orientation[0] = (int) newOrientation.firstAngle;
                orientation[1] = (int) newOrientation.secondAngle;
                orientation[2] = (int) newOrientation.thirdAngle;

                // Timestamp the update
                timestamp = System.currentTimeMillis();

                // Debug telemetry, if enabled
                if (DEBUG) {
                    for (int i = 0; i < newLocation.numRows(); i++) {
                        for (int j = 0; j < newLocation.numCols(); j++) {
                            telemetry.addData("Loc[" + i + "][" + j + "]", newLocation.get(i, j));
                        }
                    }
                    for (int i = 0; i < orientation.length; i++) {
                        telemetry.addData("Rot[" + i + "]", orientation[i]);
                    }
                    telemetry.addData("Location", newLocation.formatAsTransform());
                }
            }
        }
    }


    /**
     * Getters
     */
    public HashMap<String, Boolean> getVisible() {
        return visible;
    }

    public long getTimestamp() {
        return timestamp;
    }

    public boolean isStale() {
        return (timestamp + TRACKING_TIMEOUT < System.currentTimeMillis());
    }

    public int[] getLocation() {
        return location;
    }

    public int[] getOrientation() {
        return orientation;
    }

    public int getX() {
        return location[0];
    }

    public int getY() {
        return location[1];
    }

    public int getHeading() {
        // TODO: This is probably not the heading
        return orientation[0];
    }


    /**
     * Helpers
     */

    // It's like a macro, but for Java
    private OpenGLMatrix positionRotationMatrix(float[] postion, float[] rotation, AxesOrder order) {
        return OpenGLMatrix
                .translation(postion[0], postion[1], postion[2])
                .multiplied(Orientation.getRotationMatrix(
                        AXES_REFERENCE, order, ANGLE_UNIT,
                        rotation[0], rotation[1], rotation[2]));
    }

    // More Java blasphemy
    private VuforiaTrackable initTrackable(VuforiaTrackables trackables, int index, String name, float[] position, float[] rotation) {
        if (index > TARGETS_NUM || index < 0) {
            RobotLog.a("Invalid Vuforia trackable index (%d) for target: %s", index, name);
            return null;
        }

        VuforiaTrackable trackable = trackables.get(index);
        trackable.setName(name);
        OpenGLMatrix location = positionRotationMatrix(position, rotation, AxesOrder.XZX);
        trackable.setLocation(location);
        return trackable;
    }
}
