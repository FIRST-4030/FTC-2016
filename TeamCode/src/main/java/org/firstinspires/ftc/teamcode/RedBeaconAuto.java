package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Timer;

/**
 * Created by robotics on 11/11/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Red Beacon Auto", group = "AutoMode")
public class RedBeaconAuto extends FourWheelAutoMethods {

    Servo leftBooper;
    Servo rightBooper;
    ColorSensor frontColor;
    ColorSensor backColor;
    LightSensor light;
    ModernRoboticsI2cRangeSensor distance;
    Timer timer;

    //Not sure if these are needed
    static int[] RED_VAL_ON_WHITE;
    static int[] GRN_VAL_ON_WHITE;
    static int[] BLU_VAL_ON_WHITE;
    static int[] RED_VAL_ON_GRAY;
    static int[] GRN_VAL_ON_GRAY;
    static int[] BLU_VAL_ON_GRAY;

    public static final int WHITE_ALPHA_THRESHOLD = 15;

    static double BEACON_BLUE_MIN;
    static double BEACON_BLUE_MAX;
    static double BEACON_RED_MIN;
    static double BEACON_RED_MAX;

    static double NEAR_WALL;
    static int BOOPER_UP;
    static int BOOPER_READ;
    static int BOOPER_PUSH;

    static long TURN_TIME;
    public static int stateCounter = 1;

    @Override
    public void init() {
        //leftBooper.setPosition(BOOPER_UP);
        //rightBooper.setPosition(BOOPER_UP);
        initMotors("front left", "back left", "front right", "back right");
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontColor = hardwareMap.colorSensor.get("color sensor1");
        backColor = hardwareMap.colorSensor.get("color sensor2");
        //light = hardwareMap.lightSensor.get("light");
        distance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasound");
        distance.initialize();
        distance.enableLed(true);
        frontColor.enableLed(true);
        backColor.enableLed(true);
        //light.enableLed(false);
    }

    public boolean isColorSensorOnWhite(ColorSensor sensor) {
        return sensor.alpha() > WHITE_ALPHA_THRESHOLD;
    }

    public boolean isColorSensorOnGray(ColorSensor sensor) {
        return sensor.alpha() < WHITE_ALPHA_THRESHOLD;
    }

    long startTimeSix = 0;
    long startTimeSeven = 0;
    long startTimeEight = 0;
    boolean hasGoneThroughOnce = false;

    @Override
    public void loop() {

        switch (stateCounter) {
            case 1:
                boolean frontOnWhite = isColorSensorOnWhite(frontColor);
                boolean backOnWhite = isColorSensorOnWhite(backColor);
                if(!frontOnWhite && !backOnWhite) {
                    runMotors(1.0, 1.0);
                } else if(frontOnWhite && backOnWhite) {
                    stopMotors();
                    stateCounter = 2;
                } else if(backOnWhite && !frontOnWhite) {
                    stopMotors();
                    System.exit(1);
                } else {
                    stopMotors();
                    stateCounter = 3;
                }
                break;
            case 2:
                if(isColorSensorOnWhite(frontColor) && isColorSensorOnWhite(backColor)) {
                    runMotors(-0.5, 0.5);
                } else {
                    stopMotors();
                    stateCounter = 5;
                }
                break;
            case 3:
                if(isColorSensorOnWhite(backColor)) {
                    stopMotors();
                    stateCounter = 4;
                } else {
                    runMotors(0, 0.5);
                }
                break;
            case 4:
                if(isColorSensorOnGray(backColor)) {
                    stopMotors();
                    stateCounter = 5;
                } else {
                    runMotors(0, 0.5);
                }
                break;
            case 5:
                if(distance.getDistance(DistanceUnit.CM) < NEAR_WALL) {
                    frontOnWhite = isColorSensorOnWhite(frontColor);
                    backOnWhite = isColorSensorOnWhite(backColor);
                    if(frontOnWhite) {
                        runMotors(0.25, 0.75);
                    } else if(backOnWhite) {
                        runMotors(0.75, 0.25);
                    } else {
                        runMotors(1.0, 1.0);
                    }
                } else {
                    stopMotors();
                    leftBooper.setPosition(BOOPER_READ);
                    double lightVal = light.getRawLightDetected();
                    if(lightVal > BEACON_BLUE_MIN && lightVal < BEACON_BLUE_MAX) {
                        stateCounter = 6;
                    } else if(lightVal > BEACON_RED_MIN && lightVal < BEACON_RED_MAX) {
                        stateCounter = 7;
                    } else {
                        leftBooper.setPosition(BOOPER_UP);
                        System.exit(1);
                    }
                }
                break;
            case 6:
                if(startTimeSix == 0) {
                    startTimeSix = System.currentTimeMillis();
                }
                if(System.currentTimeMillis() - startTimeSix < 2000) {
                    leftBooper.setPosition(BOOPER_PUSH);
                } else {
                    leftBooper.setPosition(BOOPER_UP);
                    if(hasGoneThroughOnce) {
                        stateCounter = 9;
                    } else {
                        stateCounter = 8;
                    }
                }
                break;
            case 7:
                if(startTimeSeven == 0) {
                    startTimeSeven = System.currentTimeMillis();
                }
                if(System.currentTimeMillis() - startTimeSeven < 2000) {
                    leftBooper.setPosition(BOOPER_UP);
                    rightBooper.setPosition(BOOPER_PUSH);
                } else {
                    rightBooper.setPosition(BOOPER_UP);
                    if(hasGoneThroughOnce) {
                        stateCounter = 9;
                    } else {
                        stateCounter = 8;
                    }
                }
                break;
            case 8:
                if(startTimeEight == 0) {
                    startTimeEight = System.currentTimeMillis();
                }
                if(System.currentTimeMillis() - startTimeEight < 1500) {
                    runMotors(0.75, -0.75);
                } else {
                    stopMotors();
                    stateCounter = 1;
                }
                hasGoneThroughOnce = true;
                break;
            case 9:
                break;
        }
        telemetry.update();
    }
}
