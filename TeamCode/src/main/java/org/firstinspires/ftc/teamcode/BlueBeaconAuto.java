package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.Timer;

public class BlueBeaconAuto extends FourWheelAutoMethods {

    Servo leftBooper;
    Servo rightBooper;
    ColorSensor frontColor;
    ColorSensor backColor;
    LightSensor light;
    UltrasonicSensor distance;
    Timer timer;

    //Not sure if these are needed
    static int[] RED_VAL_ON_WHITE;
    static int[] GRN_VAL_ON_WHITE;
    static int[] BLU_VAL_ON_WHITE;
    static int[] RED_VAL_ON_GRAY;
    static int[] GRN_VAL_ON_GRAY;
    static int[] BLU_VAL_ON_GRAY;

    static double BEACON_BLUE_MIN;
    static double BEACON_BLUE_MAX;
    static double BEACON_RED_MIN;
    static double BEACON_RED_MAX;

    static double NEAR_WALL;
    static int BOOPER_UP;
    static int BOOPER_READ;
    static int BOOPER_PUSH;

    static long TURN_TIME;


    public void initConditions() {
        leftBooper.setPosition(BOOPER_UP);
        rightBooper.setPosition(BOOPER_UP);
        initMotors("front left", "back left", "front right", "back right");
        frontColor = hardwareMap.colorSensor.get("color sensor1");
        backColor = hardwareMap.colorSensor.get("color sensor2");
        light = hardwareMap.lightSensor.get("light");
        distance = hardwareMap.ultrasonicSensor.get("ultrasound");
        timer = new Timer();
        frontColor.enableLed(true);
        backColor.enableLed(true);
        light.enableLed(false);
    }

    //Turn the isOn methods into a single get color method
    public String getColorSensorColor(ColorSensor sensor) {
        //get values
        /*
        if(in white range) {
            return "white";
        } else if(in gray range) {
            return "gray";
        } else if(in blue range) {
            return "blue";
        } else if(in red range) {
            return "red";
        } else {
            return "unknown";
        }
         */
    }

    public boolean isColorSensorOnWhite(ColorSensor sensor) {
        boolean isOnWhite;
        //do some stuff
        return isOnWhite;
    }

    public boolean isColorSensorOnGray(ColorSensor sensor) {
        boolean isOnGray;
        //do some stuff
        return isOnGray;
    }

    public void turnToStraddle() {
        while(isColorSensorOnWhite(frontColor) && isColorSensorOnWhite(backColor)) {
            runMotors(-0.5, 0.5);
        }
        stopMotors();
    }

    public void pivotToStraddle() {
        boolean backOnLine = false;
        boolean backOverLine = false;
        while(!backOverLine) {
            runMotors(0, 0.5);
            if(isColorSensorOnWhite(backColor)) {
                backOnLine = true;
            } else if(isColorSensorOnGray(backColor) && backOnLine) {
                backOnLine = false;
                backOverLine = true;
            }
        }
        stopMotors();
    }

    public void beaconHitRoutine() throws InterruptedException {
        //Drive forward until a sensor hits the white line
        boolean frontOnWhite = isColorSensorOnWhite(frontColor);
        boolean backOnWhite = isColorSensorOnWhite(backColor);
        while(!frontOnWhite && !backOnWhite) {
            runMotors(1.0, 1.0);
            frontOnWhite = isColorSensorOnWhite(frontColor);
            backOnWhite = isColorSensorOnWhite(backColor);
        }
        stopMotors();

        //Turn in a way to straddle the line
        if(frontOnWhite && backOnWhite) {
            turnToStraddle();
        } else if(backOnWhite && !frontOnWhite) {
            System.exit(1);
        } else {
            pivotToStraddle();
        }

        //Drive forward while staying straight
        while(distance.getUltrasonicLevel() < NEAR_WALL) {
            frontOnWhite = isColorSensorOnWhite(frontColor);
            backOnWhite = isColorSensorOnWhite(backColor);
            if(frontOnWhite) {
                runMotors(0.25, 0.75);
            } else if(backOnWhite) {
                runMotors(0.75, 0.25);
            } else {
                runMotors(1.0, 1.0);
            }
        }
        stopMotors();

        //Hit beacon
        leftBooper.setPosition(BOOPER_READ);
        double lightVal = light.getRawLightDetected();
        if(lightVal > BEACON_BLUE_MIN && lightVal < BEACON_BLUE_MAX) {
            leftBooper.setPosition(BOOPER_PUSH);
            robotWait(2000);
            leftBooper.setPosition(BOOPER_UP);
        } else if(lightVal > BEACON_RED_MIN && lightVal < BEACON_RED_MAX) {
            leftBooper.setPosition(BOOPER_UP);
            rightBooper.setPosition(BOOPER_PUSH);
            robotWait(2000);
            rightBooper.setPosition(BOOPER_UP);
        } else {
            leftBooper.setPosition(BOOPER_UP);
            System.exit(1);
        }
    }

    /**
     * Override this method and place your code here.
     * <p/>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initConditions();

        beaconHitRoutine();

        //Turn and drive to next line
        driveToTime(0.75, -0.75, TURN_TIME);

        beaconHitRoutine();
    }
}