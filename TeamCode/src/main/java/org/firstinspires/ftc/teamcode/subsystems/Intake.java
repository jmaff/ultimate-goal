package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake implements Subsystem {
    private DcMotor motor;
    public enum IntakeState {
        IN,
        OUT,
        OFF
    }

    private IntakeState state = IntakeState.OFF;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "I.M");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void update() {
        switch (state) {
            case IN:
                motor.setPower(1.0);
                break;
            case OUT:
                motor.setPower(-1.0);
                break;
            case OFF:
                motor.setPower(0.0);
                break;
        }
    }

    public IntakeState getState() {
        return state;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }
}
