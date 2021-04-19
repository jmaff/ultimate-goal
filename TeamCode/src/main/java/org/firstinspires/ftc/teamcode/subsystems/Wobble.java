package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Wobble implements Subsystem {
    public static double DOWN_POWER = 0.4;
    public static double UP_POWER = 0.6;
    public static double kP = 0.01;
    public static int UP_COUNTS = 0;
    public static int DOWN_COUNTS = 725;
    public static int GRAB_COUNTS = 760;
    public static int MIN_ERROR = 10;

    public static double GRABBED = 0.83;
    public static double RELEASED = 0.35;

    DcMotor wobbleMotor;
    Servo grabber;

    double power = 0.0;
    int targetPosition = 0;

    public enum WobbleState {
        UP,
        DOWN,
        GRAB,
        OFF,
        MANUAL
    }

    public enum GrabberState {
        GRABBED,
        RELEASED
    }

    WobbleState wobbleState = WobbleState.OFF;
    GrabberState grabberState = GrabberState.RELEASED;

    public Wobble(HardwareMap hardwareMap) {
        wobbleMotor = hardwareMap.get(DcMotor.class, "W.M");
        grabber = hardwareMap.get(Servo.class, "W.G");

        wobbleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobbleMotor.setDirection(DcMotorSimple.Direction.REVERSE); // positive DROPS wobble
    }

    @Override
    public void update() {
        switch (wobbleState) {
            case UP:
                targetPosition = UP_COUNTS;
                power = (targetPosition - wobbleMotor.getCurrentPosition()) * kP;
                clampPower();
                break;
            case DOWN:
                targetPosition = DOWN_COUNTS;
                power = (targetPosition - wobbleMotor.getCurrentPosition()) * kP;
                clampPower();
                break;
            case GRAB:
                targetPosition = GRAB_COUNTS;
                power = (targetPosition - wobbleMotor.getCurrentPosition()) * kP;
                break;
            case OFF:
                power = 0;
                break;
            case MANUAL:
                break;
        }

        wobbleMotor.setPower(power);

        if (wobbleState != WobbleState.OFF && wobbleState != WobbleState.MANUAL) {
            if (Math.abs(getError()) < MIN_ERROR) {
                wobbleState = WobbleState.OFF;
            }
        }

        switch (grabberState) {
            case GRABBED:
                grabber.setPosition(GRABBED);
                break;
            case RELEASED:
                grabber.setPosition(RELEASED);
                break;
        }
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void setWobbleState(WobbleState wobbleState) {
        this.wobbleState = wobbleState;
    }

    public WobbleState getWobbleState() {
        return wobbleState;
    }

    public int getPosition() {
        return wobbleMotor.getCurrentPosition();
    }

    public GrabberState getGrabberState() {
        return grabberState;
    }

    public void setGrabberState(GrabberState grabberState) {
        this.grabberState = grabberState;
    }

    public int getError() {
        return targetPosition - getPosition();
    }

    public void clampPower() {
        if (wobbleState == WobbleState.UP) {
            if (power > 0 && power >= UP_POWER) {
                power = UP_POWER;
            } else if (power < 0 && power <= -UP_POWER) {
                power = -UP_POWER;
            }
        } else if (wobbleState == WobbleState.DOWN) {
            if (power > 0 && power >= DOWN_POWER) {
                power = DOWN_POWER;
            } else if (power < 0 && power <= -DOWN_POWER) {
                power = -DOWN_POWER;
            }
        }
    }
}
