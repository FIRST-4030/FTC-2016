package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by User on 11/18/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "AutoMode")
public class DriveForwardAuto extends LinearOpMode {

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor shooterMotor;
    private Servo blocker;
    private Servo leftBooper;
    private Servo rightBooper;

    private static final int SCORE_INCR = -1850;
    private static final int BALL_INCR = -1100;
    public static final int SHOOTER_INCR = 3700;
    public static final double SHOOTER_SPEED = 1.0;
    public static final double BLOCKER_UP = 0.0;
    public static final double BLOCKER_DOWN = 0.98;
    public static final int NUM_SHOTS = 2;

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
        leftMotor = hardwareMap.dcMotor.get("left-wheel-motor");
        rightMotor = hardwareMap.dcMotor.get("right-wheel-motor");
        blocker = hardwareMap.servo.get("blocker");
        shooterMotor = hardwareMap.dcMotor.get("shooter-motor");
        leftBooper = hardwareMap.servo.get("left-booper");
        rightBooper = hardwareMap.servo.get("right-booper");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBooper.setDirection(Servo.Direction.REVERSE);

        leftBooper.setPosition(0);
        rightBooper.setPosition(0);
        blocker.setPosition(BLOCKER_DOWN);

        waitForStart();

        int trueGoal = leftMotor.getCurrentPosition() + SCORE_INCR;
        while(leftMotor.getCurrentPosition() > trueGoal) {
            leftMotor.setPower(-1.0);
            rightMotor.setPower(-1.0);
            telemetry.update();
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        telemetry.update();
        idle();

        for(int i = 0; i < NUM_SHOTS; i++) {
            int shooterGoal = shooterMotor.getCurrentPosition() + SHOOTER_INCR;
            while(shooterMotor.getCurrentPosition() < shooterGoal) {
                blocker.setPosition(BLOCKER_UP);
                shooterMotor.setPower(SHOOTER_SPEED);
                telemetry.update();
                idle();
            }
            shooterMotor.setPower(0);
            blocker.setPosition(BLOCKER_DOWN);
            telemetry.update();
            idle();

            // Wait for the next ball to drop
            if (i < NUM_SHOTS - 1) {
                long end = System.currentTimeMillis() + 1000;
                while (end > System.currentTimeMillis()) {
                    idle();
                }
            }
        }

        trueGoal = leftMotor.getCurrentPosition() + BALL_INCR;
        while(leftMotor.getCurrentPosition() > trueGoal) {
            leftMotor.setPower(-1.0);
            rightMotor.setPower(-1.0);
            telemetry.update();
            idle();
        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        telemetry.update();
        idle();

    }
}
