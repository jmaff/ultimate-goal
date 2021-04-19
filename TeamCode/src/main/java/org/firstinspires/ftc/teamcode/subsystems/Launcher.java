package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Launcher implements Subsystem {
    public static double TA_FLAT = 0.32; //0.29
    public static double TA_DROP = 0.4;
    public static double TA_LINE = 0.37;
    private long lastIncrementTime = 0;
    public static double minTimeBetweenIncrement = 25;
    public static double incrementAmount = 0.01;

//    public static double LAUNCHER_POWER = 0.77;
    public static double LAUNCHER_POWER = 0.77;  //0.78
    public static double WOBBLE_POWER = 0.8;

    public DcMotor front;
    public DcMotor back;
    public Servo trajectoryAdjust;

    private double trajectoryPosition = TA_FLAT;

    public enum LauncherState {
        ON,
        OFF,
        TELE,
        WOBBLE
    }

    public enum AdjustmentState {
        HOLD,
        INCREMENT,
        DECREMENT
    }

    private LauncherState launcherState = LauncherState.OFF;
    private AdjustmentState adjustmentState = AdjustmentState.HOLD;

    public Launcher(HardwareMap hardwareMap) {
        front = hardwareMap.get(DcMotor.class, "L.L");
        back = hardwareMap.get(DcMotor.class, "L.R");
        trajectoryAdjust = hardwareMap.get(Servo.class, "L.T");

        back.setDirection(DcMotorSimple.Direction.REVERSE);
        front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        trajectoryAdjust.setPosition(TA_FLAT);
    }

    public LauncherState getLauncherState() {
        return launcherState;
    }

    public void setLauncherState(LauncherState launcherState) {
        this.launcherState = launcherState;
    }

    public AdjustmentState getAdjustmentState() {
        return adjustmentState;
    }

    public void setAdjustmentState(AdjustmentState adjustmentState) {
        this.adjustmentState = adjustmentState;
    }

    @Override
    public void update() {
        switch(launcherState) {
            case ON:
                setLauncherPower(LAUNCHER_POWER);
                break;
            case OFF:
                setLauncherPower(0.0);
                break;
            case TELE:
                setLauncherPower(0.9);
                break;
            case WOBBLE:
                setLauncherPower(WOBBLE_POWER);
                break;
        }

        if (System.currentTimeMillis() - lastIncrementTime >= minTimeBetweenIncrement && adjustmentState != AdjustmentState.HOLD) {
            if (adjustmentState == AdjustmentState.INCREMENT) {
                trajectoryPosition += incrementAmount;
            } else {
                trajectoryPosition -= incrementAmount;
            }
            lastIncrementTime = System.currentTimeMillis();
        }

        trajectoryAdjust.setPosition(trajectoryPosition);
    }

    public void setLauncherPower(double power) {
        front.setPower(power);
        back.setPower(power);
    }

    public double getTrajectoryPosition() {
        return trajectoryPosition;
    }

    public void setTrajectoryPosition(double trajectoryPosition) {
        this.trajectoryPosition = trajectoryPosition;
    }
}
