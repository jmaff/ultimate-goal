package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake implements Subsystem {
    public static double STICK_STOW = 0.35;
    public static double STICK_RAISED = 0.6;
    public static double STICK_DOWN = 0.7;

    private DcMotor motor;
    private Servo stickServo;
    public enum IntakeState {
        IN,
        OUT,
        OFF
    }

    public enum StickState {
        STOWED,
        RAISED,
        DOWN
    }

    private IntakeState state = IntakeState.OFF;
    private StickState stickState = StickState.STOWED;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "I.M");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        stickServo = hardwareMap.get(Servo.class, "I.S");
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

        switch (stickState) {
            case STOWED:
                stickServo.setPosition(STICK_STOW);
                break;
            case RAISED:
                stickServo.setPosition(STICK_RAISED);
                break;
            case DOWN:
                stickServo.setPosition(STICK_DOWN);
                break;
        }
    }

    public IntakeState getState() {
        return state;
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public StickState getStickState() {
        return stickState;
    }

    public void setStickState(StickState stickState) {
        this.stickState = stickState;
    }
}
