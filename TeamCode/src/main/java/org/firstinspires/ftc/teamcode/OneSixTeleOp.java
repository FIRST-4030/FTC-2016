package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.TankOpMode;

/**
 * Created by robotics on 11/5/2016.
 */

public class OneSixTeleOp extends TankOpMode {

    DcMotor collectorMotor;
    DcMotor shooterMotor;
    Servo leftBooper;
    Servo rightBooper;

    public static double BOOPER_UP;
    public static double BOOPER_OUT;
    public static final double COLLECTOR_IN = 1.0;
    public static final int SHOOTER_INCR = 200;

    private boolean isBPressed = false;
    private boolean isBackPressed = false;
    private boolean isCollectorOn = false;
    private boolean inFireRoutine = false;
    private static int shooterEncoderGoal;

    public OneSixTeleOp() {
        super("left-wheel-motor", "right-wheel-motor");
    }

    public void init() {
        super.init();

        collectorMotor = hardwareMap.dcMotor.get("collector-motor");
        shooterMotor = hardwareMap.dcMotor.get("shooter-motor");
        //leftBooper = hardwareMap.servo.get("left-booper");
        //rightBooper = hardwareMap.servo.get("right-booper");

        collectorMotor.setPower(COLLECTOR_IN);
        //leftBooper.setPosition(BOOPER_UP);
        //rightBooper.setPosition(BOOPER_UP);

        isCollectorOn = true;
    }

    public void loop() {
        super.loop();

        //Turning on and off the collector with the back button
        if(gamepad2.back) {
            isBackPressed = true;
        } else if(isBackPressed) {
            isBackPressed = false;
            if(isCollectorOn) {
                collectorMotor.setPower(0.0);
            } else {
                collectorMotor.setPower(COLLECTOR_IN);
            }
        }

        //Firing
        if(gamepad2.b) {
            isBPressed = true;
        } else if(isBPressed) {
            isBPressed = false;
            if(!inFireRoutine) {
                inFireRoutine = true;
                shooterEncoderGoal = shooterMotor.getCurrentPosition() + SHOOTER_INCR;
            }
        }

        //Firing is in a separate boolean to make the code able to move and fire at the same time
        if(inFireRoutine) {
            if(shooterMotor.getCurrentPosition() < shooterEncoderGoal) {
                shooterMotor.setPower(1.0);
            } else {
                shooterMotor.setPower(0);
                inFireRoutine = false;
            }
        }

        //Boopers!
        //Will come later as this is not important for today
    }

}
