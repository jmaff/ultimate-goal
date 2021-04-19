package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake implements Subsystem {
    public static double STICK_STOW = 0.2;
    public static double STICK_RAISED = 0.6;
    public static double STICK_DOWN = 0.7;

    public static double STICK_STOW_RIGHT = 0.5;
    public static double STICK_RAISED_RIGHT = 0.2;
    public static double STICK_DOWN_RIGHT = 0.0;

    public boolean noRight = false;

    private DcMotor motor;
    private Servo stickLeft;
    private Servo stickRight;
    public enum IntakeState {
        IN,
        OUT,
        OFF
    }

    public enum StickState {
        STOWED,
        RAISED,
        DOWN,
        RIGHT_STOW
    }

    private IntakeState state = IntakeState.OFF;
    private StickState stickState = StickState.STOWED;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "I.M");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        stickLeft = hardwareMap.get(Servo.class, "I.S");
        stickRight = hardwareMap.get(Servo.class, "I.SR");
    }

    @Override
    public void update() {
        switch (state) {
            case IN:
                motor.setPower(0.75);
                break;
            case OUT:
                motor.setPower(-1.0);
                break;
            case OFF:
                motor.setPower(0.0);
                break;
        }

        if (!noRight) {
            switch (stickState) {
                case STOWED:
                    stickLeft.setPosition(STICK_STOW);
                    stickRight.setPosition(STICK_STOW_RIGHT);
                    break;
                case RAISED:
                    stickLeft.setPosition(STICK_RAISED);
                    stickRight.setPosition(STICK_RAISED_RIGHT);
                    break;
                case DOWN:
                    stickLeft.setPosition(STICK_DOWN);
                    stickRight.setPosition(STICK_DOWN_RIGHT);
                    break;
            }
        }else {
                switch (stickState) {
                    case STOWED:
                        stickLeft.setPosition(STICK_STOW);
                        stickRight.setPosition(STICK_STOW_RIGHT);
                        break;
                    case RAISED:
                        stickLeft.setPosition(STICK_RAISED);
                        stickRight.setPosition(STICK_STOW_RIGHT);
                        break;
                    case DOWN:
                        stickLeft.setPosition(STICK_DOWN);
                        stickRight.setPosition(STICK_STOW_RIGHT);
                        break;
            }
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
