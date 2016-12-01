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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import java.util.HashMap;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends LinearOpMode {

    @SuppressWarnings("WeakerAccess")
    public static final String TAG = "Vuforia Test";

    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Init Vuforia -- it needs a team-specific license key and data about the camera
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AbgpAh3/////AAAAGTwS0imaZU6wjVVHhw7cr1iHxcyPegw1+zPNzs+oNjtZlwpyvuwb2hdTLeEEj0gPTWUgVfLbnn6BrV6pafSnN8oCEEZrbVicTGw02BT+V0IzD43++kcsLVuumaM9yAUlAaDPiuEvEx6AZxYnM05KMzlAtMtfgW8tOIvjlicxep9tPhr1Z1Z3JrDt8s8mPo3GsSRSvpoSXZfxRLi0CwGEJlTuVrP59wLhsvr3CZ5Nr7gCNznhAaiGp4LhtCPoXsIUjsQHwO2hmskW670gZGIZl7BvqVbN5mIwqOYF3ZsCUkR83pM7jSIsOMdiaLK5ZlVLG+z5AfgoPNDZo8iYiqTncIiSUL5oJuh2NIeiG+nwcPJV";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Pre-processed target images from the Vuforia target manager:
         * https://developer.vuforia.com/target-manager.
         */
        VuforiaTrackables wheelsToolsLegoGears = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        /**
         * Name the 4 targets and grab references to each
         * TODO: This should be a data structure
         */
        VuforiaTrackable redTargetA = wheelsToolsLegoGears.get(0);
        redTargetA.setName("Wheels");
        VuforiaTrackable redTargetB = wheelsToolsLegoGears.get(1);
        redTargetB.setName("Tools");
        VuforiaTrackable blueTargetA = wheelsToolsLegoGears.get(2);
        blueTargetA.setName("LEGO");
        VuforiaTrackable blueTargetB = wheelsToolsLegoGears.get(3);
        blueTargetB.setName("Gears");

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<>();
        allTrackables.addAll(wheelsToolsLegoGears);

        /**
         * Record the visibility of all targets during each Vuforia processing loop
         */
        HashMap<String, Boolean> visibleTargets = new HashMap<>();
        for (VuforiaTrackable trackable : allTrackables) {
            visibleTargets.put(trackable.getName(), false);
        }

        // Field and robot parameters
        // TODO: These values are approximate. We should improve them before navigation.
        float mmPerInch = 25.4f;
        float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        // Red targets
        // TODO: These positions are imaginary. We need to find the real ones before navigation.
        setTargetPosition(-mmFTCFieldWidth / 2, 0, 0, redTargetA);
        setTargetPosition(-mmFTCFieldWidth / 2, mmFTCFieldWidth / 4, 0, redTargetB);

        // Blue targets
        // TODO: These positions are imaginary. We need to find the real ones before navigation.
        setTargetPosition(0, mmFTCFieldWidth / 2, 0, blueTargetA);
        setTargetPosition(mmFTCFieldWidth / 4, mmFTCFieldWidth / 2, 0, blueTargetB);

        /**
         * Let Vuforia know that we want to track all targets relative to the phone and provide a
         * transformation to defien the phone's location relative to the robot
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth / 2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // Wait for the game to begin
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        // Start Vuforia tracking
        wheelsToolsLegoGears.activate();

        long lastTimestamp = 0;
        OpenGLMatrix lastLocation = null;

        while (opModeIsActive()) {
            /**
             * Track all configured targets
             */
            for (VuforiaTrackable trackable : allTrackables) {
                // Visibility updates for all targets and status updates for the tracked target
                boolean visible = ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible();
                visibleTargets.put(trackable.getName(), visible);

                // If we're evaluating an enabled target, update the location track
                OpenGLMatrix newLocation = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                if (newLocation != null) {
                    lastLocation = newLocation;
                    lastTimestamp = System.currentTimeMillis();
                }
            }

            /**
             * Tell the DS where were are
             */
            {
                String visible = "";
                for (String name : visibleTargets.keySet()) {
                    if (visibleTargets.get(name)) {
                        visible += name + " ";
                    }
                }
                telemetry.addData("Visible", visible);
            }
            //telemetry.addData("Pos", lastLocation != null ? lastLocation.formatAsTransform() : "<Unknown>");
            telemetry.addData("X", lastLocation != null ? (int)lastLocation.get(0,3) : "<Unknonw>");
            telemetry.addData("Y", lastLocation != null ? (int)lastLocation.get(1,3) : "<Unknown>");
            //telemetry.addData("Z", lastLocation != null ? (int)lastLocation.get(2,3) : "");
            telemetry.addData("Update", lastTimestamp);

            // Loop invariants
            telemetry.update();
            idle();
        }
    }

    private void setTargetPosition(float x, float y, float z, VuforiaTrackable trackable) {
        OpenGLMatrix targetLocation = OpenGLMatrix
                .translation(x, y, z)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        trackable.setLocation(targetLocation);
    }
}
