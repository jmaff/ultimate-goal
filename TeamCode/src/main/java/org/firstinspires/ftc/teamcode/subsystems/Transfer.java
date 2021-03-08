package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Transfer implements Subsystem {
    public static double PIVOT_UP = 0.6;
    public static double PIVOT_DOWN = 0.83;

    public static double FLICKER_POWER = 0.27;

    public static double SAFETY_ENGAGED = 0.34;
    public static double SAFETY_DISENGAGED = 0.0;


    private Servo pivot;
    public CRServo flicker;
    private DigitalChannel limit;
    private Servo safety;

    public enum PivotState {
        UP,
        DOWN,
    }

    public enum FlickerState {
        FIRE,
        TELE,
        OFF,
        GO_TO_LIMIT,
        GO_TO_LIMIT_REVERSE,
        FIRE_ONE,
        MANUAL
    }

    public enum SafetyState {
        ENGAGED,
        DISENGAGED
    }

    private PivotState pivotState = PivotState.DOWN;
    private FlickerState flickerState = FlickerState.OFF;
    private FlickerState previousFlickerState = FlickerState.OFF;
    private SafetyState safetyState = SafetyState.DISENGAGED;
    long startFireOne = 0;

    boolean startingSingleFire = false;

    public Transfer(HardwareMap hardwareMap) {
        pivot = hardwareMap.get(Servo.class, "T.P");
        flicker = hardwareMap.get(CRServo.class, "T.F");
        limit = hardwareMap.get(DigitalChannel.class, "T.M");
        flicker.setDirection(DcMotorSimple.Direction.REVERSE);

        safety = hardwareMap.get(Servo.class, "T.S");
    }

    public PivotState getPivotState() {
        return pivotState;
    }

    public void setPivotState(PivotState pivotState) {
        this.pivotState = pivotState;
    }

    public FlickerState getFlickerState() {
        return flickerState;
    }

    public void setFlickerState(FlickerState flickerState) {
        this.flickerState = flickerState;
    }

    public SafetyState getSafetyState() {
        return safetyState;
    }

    public void setSafetyState(SafetyState safetyState) {
        this.safetyState = safetyState;
    }

    @Override
    public void update() {
        switch (pivotState) {
            case UP:
                pivot.setPosition(PIVOT_UP);
                break;
            case DOWN:
                pivot.setPosition(PIVOT_DOWN);
                break;
        }

        switch (flickerState) {
            case TELE:
                flicker.setPower(0.32);
                break;
            case FIRE:
                flicker.setPower(FLICKER_POWER);
                break;
            case OFF:
                flicker.setPower(0.0);
                break;
            case GO_TO_LIMIT:
                if (limit.getState()) {
                    flicker.setPower(0.15);
                } else {
                    flickerState = FlickerState.OFF;
                }
                break;
            case GO_TO_LIMIT_REVERSE:
                if (limit.getState()) {
                    flicker.setPower(-0.15);
                } else {
                    flickerState = FlickerState.OFF;
                }
                break;
            case FIRE_ONE:
                if (previousFlickerState != FlickerState.FIRE_ONE) {
                    startFireOne = System.currentTimeMillis();
                    flicker.setPower(0.2);
                }

                if (limit.getState()) {
                    flicker.setPower(0.2);
                } else {
                    if (System.currentTimeMillis() - startFireOne > 300) {
                        flickerState = FlickerState.OFF;
                    }
                }
                break;
            case MANUAL:
                break;
        }

        switch (safetyState) {
            case ENGAGED:
                safety.setPosition(SAFETY_ENGAGED);
                break;
            case DISENGAGED:
                safety.setPosition(SAFETY_DISENGAGED);
                break;
        }

        previousFlickerState = flickerState;
    }
}
