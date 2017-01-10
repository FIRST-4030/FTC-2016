package org.firstinspires.ftc.teamcode.wheels;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TankMotor {
    public MotorSide side;
    public String name;
    public DcMotor motor;
    public boolean reverse;

    public TankMotor(String name, MotorSide side) {
        this(name, side, false);
    }

    public TankMotor(String name, MotorSide side, boolean reverse) {
        this.name = name;
        this.side = side;
        this.motor = null;
        this.reverse = reverse;
    }

    public boolean isAvailable() {
        return this.motor != null;
    }
}
