package org.firstinspires.ftc.teamcode.classes;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public class VuforiaTarget {
    private static final int NUM_DIMENSIONS = 3;

    public final String name;
    public final float[] raw;
    public final float[] offset;
    public final float[] rotation;
    public final AxesOrder axesOrder;
    public final int[] adjusted;

    public VuforiaTarget(String name, float[] location, float[] offset, float[] rotation, AxesOrder axesOrder) {
        this.name = name;
        this.raw = location;
        if (offset != null) {
            this.offset = offset;
        } else {
            this.offset = new float[NUM_DIMENSIONS];
        }
        this.rotation = rotation;
        if (axesOrder != null) {
            this.axesOrder = axesOrder;
        } else {
            this.axesOrder = AxesOrder.XZX;
        }

        this.adjusted = new int[NUM_DIMENSIONS];
        for (int i = 0; i < NUM_DIMENSIONS; i++) {
            assert offset != null;
            this.adjusted[i] = (int) (location[i] + offset[i]);
        }
    }

    public VuforiaTarget(String name, float[] location, float[] offset, float[] rotation) {
        this(name, location, offset, rotation, null);
    }

    public VuforiaTarget(String name, float[] location, float[] rotation, AxesOrder axesOrder) {
        this(name, location, null, rotation, axesOrder);
    }
}