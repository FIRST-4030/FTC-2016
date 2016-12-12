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

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.classes.TankOpMode;
import org.firstinspires.ftc.teamcode.classes.VuforiaFTC;
import org.firstinspires.ftc.teamcode.classes.VuforiaTarget;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Vuforia Test", group = "Test")
public class VuforiaTest extends TankOpMode {

    public static final String TAG = "Vuforia Test";

    // Field, camera and robot constants
    public static final float MM_PER_INCH = 25.4f;
    public static final float BOT_WIDTH = 18 * MM_PER_INCH;
    public static final float FIELD_WIDTH = (12 * 12 - 2) * MM_PER_INCH;

    // Tracking config
    public static final String CONFIG_ASSET = "FTC_2016-17";
    public static final int CONFIG_TARGET_NUM = 4;
    // TODO: This location and rotation is imaginary, but should at least be close.
    private static final VuforiaTarget CONFIG_PHONE = new VuforiaTarget(
            "Phone",
            new float[]{BOT_WIDTH / 2, 0, 0},
            new float[]{-90, 0, 0},
            AxesOrder.YZY
    );
    // TODO: These locations are imaginary. We need to find the real ones before navigation.
    private static final float[] TARGETS_ROTATION_RED = {270, 180, 0};
    private static final float[] TARGETS_ROTATION_BLUE = {270, 270, 0};
    private static final float[] TARGETS_OFFSET_RED = {250, 0, 0};
    private static final float[] TARGETS_OFFSET_BLUE = {0, -250, 0};
    private static final float TARGETS_Y_BLUE = FIELD_WIDTH / 2;
    private static final float TARGETS_X_RED = FIELD_WIDTH / 2;
    private static final VuforiaTarget[] CONFIG = {new VuforiaTarget(
            "Wheels",
            new float[]{100, TARGETS_Y_BLUE, 0},
            TARGETS_OFFSET_BLUE,
            TARGETS_ROTATION_BLUE
    ), new VuforiaTarget(
            "Tools",
            new float[]{TARGETS_X_RED, 600, 0},
            TARGETS_OFFSET_RED,
            TARGETS_ROTATION_RED
    ), new VuforiaTarget(
            "LEGO",
            new float[]{-600, TARGETS_Y_BLUE, 0},
            TARGETS_OFFSET_BLUE,
            TARGETS_ROTATION_BLUE
    ), new VuforiaTarget(
            "Gears",
            new float[]{TARGETS_X_RED, -100, 0},
            TARGETS_OFFSET_RED,
            TARGETS_ROTATION_RED
    )};

    // Driving constants
    private final float ENCODER_PER_MM = 1.0f;
    private final int DRIVE_MAX_TURN = 15;

    // Dynamic things we need to remember
    private boolean DISABLE_TELEOP = false;
    private VuforiaFTC VUFORIA;

    // Tank Drive constructor
    @SuppressWarnings("unused")
    public VuforiaTest() {
        super("front left", "front right", "back left", "back right");
    }

    @Override
    public void init() {

        // Placate drivers; sometimes VuforiaFTC is slow to init
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
        VUFORIA = new VuforiaFTC(CONFIG_ASSET, CONFIG_TARGET_NUM, CONFIG, CONFIG_PHONE);
        VUFORIA.init();

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
        VUFORIA.start();
    }

    @Override
    public void loop() {
        // Inherit TankDrive functionality (if enabled)
        if (!DISABLE_TELEOP) {
            super.loop();
        }

        // Update our location and pose
        VUFORIA.track();
        VUFORIA.display(telemetry);

        // Turn to face the first visible target and drive half the distance toward it
        if (gamepad1.y && !VUFORIA.isStale()) {
            boolean valid = false;
            int bearing = 0;
            int distance = 0;

            // Get data for the first visible target
            for (String target : VUFORIA.getVisible().keySet()) {
                if (VUFORIA.getVisible(target)) {
                    int index = VUFORIA.getTargetIndex(target);
                    bearing = VUFORIA.bearing(index);
                    distance = VUFORIA.distance(index);
                    valid = true;
                    break;
                }
            }

            // If we're tracking a target
            if (valid) {
                int ticks = (int) (distance * ENCODER_PER_MM) / 2;
                int angle = VUFORIA.getHeading() - bearing;

                // Drive only if the turn is likely to keep us at a tracking angle
                if (Math.abs(angle) < DRIVE_MAX_TURN) {
                    // TODO: Something that makes us turn <angle> degrees and then drive <ticks> encoder ticks
                }
            }

            // Loop invariants
            telemetry.update();
        }
    }
}
