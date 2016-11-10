package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Timer;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Blue Beacon Auto", group = "AutoMode")
public class BlueBeaconAuto extends FourWheelAutoMethods {

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


    public void initConditions() {
        //leftBooper.setPosition(BOOPER_UP);
        //rightBooper.setPosition(BOOPER_UP);
        initMotors("front left", "back left", "front right", "back right");
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontColor = hardwareMap.colorSensor.get("color sensor1");
        backColor = hardwareMap.colorSensor.get("color sensor2");
        //light = hardwareMap.lightSensor.get("light");
        distance = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "ultrasound");
        distance.initialize();
        timer = new Timer();
        distance.enableLed(true);
        frontColor.enableLed(true);
        backColor.enableLed(true);
        //light.enableLed(false);
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
        } else {*/
            return "unknown";
        /*}
         */
    }

    public boolean isColorSensorOnWhite(ColorSensor sensor) {
        return sensor.alpha() > WHITE_ALPHA_THRESHOLD;
    }

    public boolean isColorSensorOnGray(ColorSensor sensor) {
        return sensor.alpha() < WHITE_ALPHA_THRESHOLD;
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
            frontLeftMotor.setPower(1.0);
            backLeftMotor.setPower(1.0);
            frontRightMotor.setPower(1.0);
            backRightMotor.setPower(1.0);
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

        System.exit(0);

        //Drive forward while staying straight
        while(distance.cmUltrasonic() < NEAR_WALL) {
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

        // Wait for driver station start
        waitForStart();

        /*
        while(opModeIsActive()) {
            telemetry.addData("Range", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("Range Optical", distance.rawOptical());
            telemetry.addData("RGB 1", "%03d %03d %03d", frontColor.red(), frontColor.green(), frontColor.blue());
            telemetry.addData("RGB 2", "%03d %03d %03d", backColor.red(), backColor.green(), backColor.blue());
            telemetry.update();
            idle();
        }
        */

        beaconHitRoutine();

        //Turn and drive to next line
        //driveToTime(0.75, -0.75, TURN_TIME);

        //beaconHitRoutine();
    }
}