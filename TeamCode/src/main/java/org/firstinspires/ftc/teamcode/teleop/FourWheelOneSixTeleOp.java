package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ariel.TankOpMode;

@SuppressWarnings("unused")
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Four-Wheeled Tele-Op", group = "Teleop")
public class FourWheelOneSixTeleOp extends TankOpMode {

    private DcMotor collectorMotor;
    private DcMotor shooterMotor;
    private Servo leftBooper;
    private Servo rightBooper;
    private Servo blocker;
    private Servo flapper;

    private static final double BOOPER_IN = 0.0;
    private static final double MAX_BOOPER_OUT = 0.35;
    private static final double BOOPER_CALIBRATE = 0.43;
    private static final double BOOPER_INCR = 0.005;
    private static final double BLOCKER_UP = 0.0;
    private static final double BLOCKER_DOWN = 0.98;
    private static final double FLAPPER_UP = 0.70;
    private static final double FLAPPER_DOWN = 0.0;
    private static final double COLLECTOR_IN = 1.0;
    private static final double SHOOTER_SPEED = 1.0;
    private static final int SHOOTER_INCR = 3550;

    private boolean isA1Pressed = false;
    private boolean inFireRoutine = false;
    private int shooterEncoderGoal;
    private double leftBooperPosition;
    private double rightBooperPosition;

    @SuppressWarnings("unused")
    public FourWheelOneSixTeleOp() {
        super("left-wheel-motor", "right-wheel-motor", "left-back-motor", "right-back-motor");
    }

    public void init() {
        super.init();

        collectorMotor = hardwareMap.dcMotor.get("collector-motor");
        shooterMotor = hardwareMap.dcMotor.get("shooter-motor");
        leftBooper = hardwareMap.servo.get("left-booper");
        rightBooper = hardwareMap.servo.get("right-booper");
        blocker = hardwareMap.servo.get("blocker");
        flapper = hardwareMap.servo.get("flapper");
        rightBooper.setDirection(Servo.Direction.REVERSE);

        setLeftBooperPosition(0);
        setRightBooperPosition(0);
        blocker.setPosition(BLOCKER_DOWN);
    }

    public void loop() {
        super.loop();

        CollectTelemetry();

        //Emergency out and collector off
        if(gamepad2.back) {
            collectorMotor.setPower(-COLLECTOR_IN);
        } else if(gamepad2.a) {
            collectorMotor.setPower(0.0);
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

        if(gamepad1.dpad_up) {
            if(leftBooperPosition + BOOPER_INCR <= MAX_BOOPER_OUT) {
                setLeftBooperPosition(leftBooperPosition + BOOPER_INCR);
            } else {
                setLeftBooperPosition(MAX_BOOPER_OUT);
            }
            if(rightBooperPosition + BOOPER_INCR <= MAX_BOOPER_OUT) {
                setRightBooperPosition(rightBooperPosition + BOOPER_INCR);
            } else {
                setRightBooperPosition(MAX_BOOPER_OUT);
            }
        } else if(gamepad1.dpad_down) {
            if(leftBooperPosition - BOOPER_INCR >= BOOPER_IN) {
                setLeftBooperPosition(leftBooperPosition - BOOPER_INCR);
            } else {
                setLeftBooperPosition(BOOPER_IN);
            }
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

        //Hold for flapper down, up by default
        if(gamepad1.right_bumper) {
            flapper.setPosition(FLAPPER_DOWN);
        } else {
            flapper.setPosition(FLAPPER_UP);
        }
    }

    public float moderateMotorPower(float motorPower) {
        float power = super.moderateMotorPower(motorPower);

        if(gamepad1.left_bumper) {
            power /= 2;
        }

        return power;
    }

    private void setLeftBooperPosition(double position) {
        leftBooper.setPosition(position);
        leftBooperPosition = position;
    }

    private void setRightBooperPosition(double position) {
        rightBooper.setPosition(position);
        rightBooperPosition = position;
    }

    public void CollectTelemetry() {
        telemetry.addData("Left Booper", leftBooper.getPosition());
        telemetry.addData("Right Booper", rightBooper.getPosition());
        telemetry.addData("Blocker", blocker.getPosition());
        telemetry.addData("Flapper", flapper.getPosition());
        telemetry.addData("Shooter Motor", shooterMotor.getCurrentPosition());
        telemetry.addData("Left Front Wheel", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Right Front Wheel", frontRightMotor.getCurrentPosition());
        telemetry.update();
    }

}
