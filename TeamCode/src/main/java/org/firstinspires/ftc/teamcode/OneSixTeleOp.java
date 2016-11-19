package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.TankOpMode;

/**
 * Created by robotics on 11/5/2016.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele-Op", group = "Iterative Opmode")
public class OneSixTeleOp extends TankOpMode {

    DcMotor collectorMotor;
    DcMotor shooterMotor;
    Servo leftBooper;
    Servo rightBooper;
    Servo blocker;

    public static final double BOOPER_IN = 0.0;
    public static final double MAX_BOOPER_OUT = 0.35;
    public static final double BOOPER_CALIBRATE = 0.43;
    public static final double BOOPER_INCR = 0.005;
    public static final double BLOCKER_UP = 0.98;
    public static final double BLOCKER_DOWN = 0.17;
    public static final double COLLECTOR_IN = 1.0;
    public static final double SHOOTER_SPEED = 1.0;
    public static final int SHOOTER_INCR = 3700;

    private boolean isA1Pressed = false;
    private boolean isA2Pressed = false;
    private boolean isBackPressed = false;
    private boolean isCollectorOn = false;
    private boolean inFireRoutine = false;
    private int shooterEncoderGoal;
    private double leftBooperPosition;
    private double rightBooperPosition;
    private double blockerPosition;

    public OneSixTeleOp() {
        super("left-wheel-motor", "right-wheel-motor");
    }

    public void init() {
        super.init();

        collectorMotor = hardwareMap.dcMotor.get("collector-motor");
        shooterMotor = hardwareMap.dcMotor.get("shooter-motor");
        leftBooper = hardwareMap.servo.get("left-booper");
        rightBooper = hardwareMap.servo.get("right-booper");
        blocker = hardwareMap.servo.get("blocker");
        rightBooper.setDirection(Servo.Direction.REVERSE);

        setLeftBooperPosition(0);
        setRightBooperPosition(0);
        blocker.setPosition(BLOCKER_DOWN);
        blockerPosition = 1.0;

        isCollectorOn = false;
    }

    public void loop() {
        super.loop();

        CollectTelemetry();

        //Emergency out and collector off
        if(gamepad2.back) {
            collectorMotor.setPower(-COLLECTOR_IN);
            isBackPressed = true;
        } else if(gamepad2.a) {
            collectorMotor.setPower(0.0);
            isA2Pressed = true;
        } else {
            collectorMotor.setPower(COLLECTOR_IN);
        }

        //Firing
        if(gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
            inFireRoutine = true;
            shooterEncoderGoal = shooterMotor.getCurrentPosition() + SHOOTER_INCR;
        }

        //Firing is in a separate boolean to make the code able to move and fire at the same time
        if(inFireRoutine) {
            if(shooterMotor.getCurrentPosition() < shooterEncoderGoal) {
                blocker.setPosition(BLOCKER_UP);
                shooterMotor.setPower(SHOOTER_SPEED);
            } else {
                shooterMotor.setPower(0);
                blocker.setPosition(BLOCKER_DOWN);
                inFireRoutine = false;
            }
        }

        //Boopers!
        if(gamepad1.a) {
            isA1Pressed = true;
        } else if(isA1Pressed) {
            isA1Pressed = false;
            setLeftBooperPosition(BOOPER_IN);
            setRightBooperPosition(BOOPER_IN);
        }

        if(gamepad1.left_trigger > 0.5) {
            if(leftBooperPosition + BOOPER_INCR <= MAX_BOOPER_OUT) {
                setLeftBooperPosition(leftBooperPosition + BOOPER_INCR);
            } else {
                setLeftBooperPosition(MAX_BOOPER_OUT);
            }
        } else if(gamepad1.left_bumper) {
            if(leftBooperPosition - BOOPER_INCR >= BOOPER_IN) {
                setLeftBooperPosition(leftBooperPosition - BOOPER_INCR);
            } else {
                setLeftBooperPosition(BOOPER_IN);
            }
        }

        if(gamepad1.right_trigger > 0.5) {
            if(rightBooperPosition + BOOPER_INCR <= MAX_BOOPER_OUT) {
                setRightBooperPosition(rightBooperPosition + BOOPER_INCR);
            } else {
                setRightBooperPosition(MAX_BOOPER_OUT);
            }
        } else if(gamepad1.right_bumper) {
            if(rightBooperPosition - BOOPER_INCR >= BOOPER_IN) {
                setRightBooperPosition(rightBooperPosition - BOOPER_INCR);
            } else {
                setRightBooperPosition(BOOPER_IN);
            }
        }

        if(gamepad1.y && gamepad1.a) {
            setLeftBooperPosition(BOOPER_CALIBRATE);
            setRightBooperPosition(BOOPER_CALIBRATE);
        }
    }

    public void setLeftBooperPosition(double position) {
        leftBooper.setPosition(position);
        leftBooperPosition = position;
    }

    public void setRightBooperPosition(double position) {
        rightBooper.setPosition(position);
        rightBooperPosition = position;
    }

    public void CollectTelemetry() {
        telemetry.addData("Left Booper", leftBooper.getPosition());
        telemetry.addData("Right Booper", rightBooper.getPosition());
        telemetry.addData("Blocker", blocker.getPosition());
        telemetry.addData("Shooter Motor", shooterMotor.getCurrentPosition());
        telemetry.addData("Left Wheel", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Right Wheel", frontRightMotor.getCurrentPosition());
        telemetry.update();
    }

}
