package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.vuforia.VuforiaFTC;
import org.firstinspires.ftc.teamcode.wheels.TankDrive;

public class Navigation {
    private final OpMode op;
    private final TankDrive tank;
    private final VuforiaFTC vuforia;
    private final Gyro gyro;

    private boolean headingValid;
    private double headingSyncExpires;

    public static final int FULL_CIRCLE = 360;

    public Navigation(OpMode op, TankDrive tank, VuforiaFTC vuforia, Gyro gyro) {
        this.op = op;
        this.tank = tank;
        this.vuforia = vuforia;
        this.gyro = gyro;

        this.headingValid = false;
        this.headingSyncExpires = 0;
    }

    public void setTeleop(boolean enable) {
        tank.setTeleop(enable);
    }

    public void loop(Gamepad pad) {
        tank.loop(pad);
        vuforia.track();
    }

    public void loop() {
        loop(null);
    }

    public void turnAngle(int angle) {
        setTeleop(false);
        //DriveToParams param = new DriveToParams(this, VuforiaAuto.SENSOR_TYPE.GYRO);
        //param.timeout = (Math.abs(angle) * TIMEOUT_DEGREE) + TIMEOUT_DEFAULT;

        // Normalized heading and bearing
        int target = gyro.getHeading() + angle;

        // Turn CCW for negative angles
        if (angle > 0) {
            //param.greaterThan(target - OVERRUN_GYRO);
        } else {
            //param.lessThan(target + OVERRUN_GYRO);
        }
        // TODO: Something like DriveTo but interruptable for tracking
        //drive = new DriveTo(new DriveToParams[]{param});
    }

    public void turnHeading(int target) {
        // Normalized heading and turns in each direction
        int current = gyro.getHeadingBasic();
        int cw = (target - current + FULL_CIRCLE) % FULL_CIRCLE;
        int ccw = (current - target + FULL_CIRCLE) % FULL_CIRCLE;

        // Turn the short way
        if (Math.abs(cw) <= Math.abs(ccw)) {
            // TODO: Turn CW, monitoring our position as we go
        } else {
            // TODO: Turn -CCW, monitoring our position as we go
        }
    }

    public boolean findTrackable(NavHint hint) {
        if (hint == null || hint.heading == null) {
            // TODO: No heading hint
        }
        if (!headingValid) {
            // TODO: No current heading
        } else if (op.time > headingSyncExpires) {
            // TODO: Weak current heading
        }
        return false;
    }

    // TODO: Something to find the closest tracking target *by heading and distance*
    /*
    private int closestTarget(Field.AllianceColor color) {
        int index = -1;
        int min = Integer.MAX_VALUE;
        for (int i = 0; i < config.length; i++) {
            if (config[i].color.equals(color)) {
                int distance = vuforia.distance(config[i].adjusted[0], config[i].adjusted[1]);
                if (distance < min) {
                    min = distance;
                    index = i;
                }
            }
        }
        return index;
    }
    */
}
