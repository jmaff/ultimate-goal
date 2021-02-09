package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Transfer implements Subsystem {
    public static double PIVOT_UP = 0.4;
    public static double PIVOT_DOWN = 0.7;
    public static double PIVOT_HOLD = 0.5;

    private long lastDownTime = 0;
    private PivotState prevPivotState = PivotState.OFF;

    private Servo pivot;
    private CRServo flicker;

    public enum PivotState {
        UP,
        DOWN,
        OFF
    }

    public enum FlickerState {
        FIRE,
        OFF
    }

    private PivotState pivotState = PivotState.DOWN;
    private FlickerState flickerState = FlickerState.OFF;

    public Transfer(HardwareMap hardwareMap) {
        pivot = hardwareMap.get(Servo.class, "T.P");
        flicker = hardwareMap.get(CRServo.class, "T.F");
        flicker.setDirection(DcMotorSimple.Direction.REVERSE);
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

    @Override
    public void update() {
        switch (pivotState) {
            case UP:
                pivot.setPosition(PIVOT_UP);
                break;
            case DOWN:
                pivot.setPosition(PIVOT_DOWN);
                if (prevPivotState != PivotState.DOWN) {
                    lastDownTime = System.currentTimeMillis();
                }
                break;
            case OFF:
                pivot.setPosition(PIVOT_HOLD);
        }

        if (System.currentTimeMillis() - lastDownTime > 1000 && pivotState == PivotState.DOWN) {
            pivotState = PivotState.OFF;
        }

        prevPivotState = pivotState;

        switch (flickerState) {
            case FIRE:
                flicker.setPower(1.0);
                break;
            case OFF:
                flicker.setPower(0.0);
                break;
        }
    }
}
