package org.firstinspires.ftc.teamcode.navigation;

public class NavHint {
    public static final int DEFAULT_POSITION_ERR = 125;
    public static final int DEFAULT_HEADING_ERR = 15;

    public Integer[] position;
    public Integer positionErr;
    public Integer heading;
    public Integer headingErr;

    public NavHint(Integer[] position, Integer positionErr, Integer heading, Integer headingErr) {
        this.position = position;
        if (positionErr != null) {
            this.positionErr = positionErr;
        } else {
            this.positionErr = DEFAULT_POSITION_ERR;
        }
        this.heading = heading;
        if (headingErr != null) {
            this.headingErr = headingErr;
        } else {
            this.headingErr = DEFAULT_HEADING_ERR;
        }
    }

    public NavHint() {
        this(null, null, null, null);
    }

    public NavHint(int x, int y, int heading) {
        this(new Integer[]{x, y}, null, heading, null);
    }

    public NavHint(int x, int y) {
        this(new Integer[]{x, y}, null, null, null);
    }

    public NavHint(int heading) {
        this(null, null, heading, null);
    }
}
