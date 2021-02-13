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
    public static double PIVOT_DOWN = 0.76;

    public static double FLICKER_POWER = 0.27;


    private Servo pivot;
    private CRServo flicker;
    private DigitalChannel limit;

    public enum PivotState {
        UP,
        DOWN,
    }

    public enum FlickerState {
        FIRE,
        OFF,
        GO_TO_LIMIT
    }

    private PivotState pivotState = PivotState.DOWN;
    private FlickerState flickerState = FlickerState.OFF;

    public Transfer(HardwareMap hardwareMap) {
        pivot = hardwareMap.get(Servo.class, "T.P");
        flicker = hardwareMap.get(CRServo.class, "T.F");
        limit = hardwareMap.get(DigitalChannel.class, "T.M");
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
                break;
        }

        switch (flickerState) {
            case FIRE:
                flicker.setPower(FLICKER_POWER);
                break;
            case OFF:
                flicker.setPower(0.0);
                break;
            case GO_TO_LIMIT:
                if (!limit.getState()) {
                    flicker.setPower(0.18);
                } else {
                    flickerState = FlickerState.OFF;
                }

        }
    }
}
