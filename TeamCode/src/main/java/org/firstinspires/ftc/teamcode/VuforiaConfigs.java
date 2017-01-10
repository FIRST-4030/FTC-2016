package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.classes.AllianceColor;
import org.firstinspires.ftc.teamcode.classes.VuforiaTarget;

public class VuforiaConfigs {
    private static final float MM_PER_INCH = 25.4f;

    static public String AssetName(int year) {
        String asset = null;

        switch(year) {
            case 2016:
                asset = "FTC_2016-17";
                break;
        }

        return asset;
    }

    static public int TargetCount(int year) {
        int count = 0;

        switch(year) {
            case 2016:
                count = 4;
                break;
        }

        return count;
    }

    static public VuforiaTarget Bot(int year) {
        VuforiaTarget config = null;

        switch (year) {
            case 2016:
                // TODO: This location and rotation is imaginary, but should at least be close.
                config = new VuforiaTarget(
                        "Phone", null,
                        new float[]{(18 * MM_PER_INCH) / 2, 0, 0},
                        new float[]{-90, 0, 0},
                        AxesOrder.YZY
                );
                break;
        }

        return config;
    }

    static public VuforiaTarget[] Field(int year) {
        VuforiaTarget[] config = null;

        switch (year) {
            case 2016:
                // TODO: These locations and rotations are imaginary.
                // We need to find the real ones for valid x/y navigation.
                float[] TARGETS_ROTATION_RED = {-90, 270, 0};
                float[] TARGETS_ROTATION_BLUE = {90, 0, 0};
                float[] TARGETS_OFFSET_RED = {250, 0, 0};
                float[] TARGETS_OFFSET_BLUE = {0, -250, 0};

                int FIELD_WIDTH = (int) ((12 * 12 - 2) * MM_PER_INCH);
                int TARGETS_Y_BLUE = FIELD_WIDTH / 2;
                int TARGETS_X_RED = -FIELD_WIDTH / 2;
                int TARGETS_OFFSET_NEAR = (int) (12 * MM_PER_INCH);
                int TARGETS_OFFSET_FAR = (int) (36 * MM_PER_INCH);

                config = new VuforiaTarget[] {new VuforiaTarget(
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

                break;
        }

        return config;
    }
}
