package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends RobotOpMode {

    @Override
    public void loop() {
        super.loop();
        // Drive
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        // Intake
        if (gamer1.RIGHT_BUMPER.pressed()) {
            if (intake.getState() != Intake.IntakeState.IN) {
                intake.setState(Intake.IntakeState.IN);
                transfer.setPivotState(Transfer.PivotState.DOWN);
                transfer.setFlickerState(Transfer.FlickerState.OFF);
            } else {
                intake.setState(Intake.IntakeState.OFF);
            }
        }

        if (gamer1.LEFT_BUMPER.pressed()) {
            if (intake.getState() != Intake.IntakeState.OUT) {
                intake.setState(Intake.IntakeState.OUT);
            } else {
                intake.setState(Intake.IntakeState.OFF);
            }
        }

        // Transfer
        if (gamer2.RIGHT_BUMPER.pressed()) {
            transfer.setPivotState(Transfer.PivotState.UP);
            transfer.setFlickerState(Transfer.FlickerState.OFF);
        }
        if (gamer2.LEFT_BUMPER.pressed()) {
            transfer.setPivotState(Transfer.PivotState.DOWN);
            transfer.setFlickerState(Transfer.FlickerState.OFF);
        }

        // Launcher
        if (gamer2.A.pressed()) {
            if (launcher.getLauncherState() != Launcher.LauncherState.ON) {
                launcher.setLauncherState(Launcher.LauncherState.ON);
            } else {
                launcher.setLauncherState(Launcher.LauncherState.OFF);
            }
        }

        // Trajectory Adjust
        if (gamer2.DPAD_UP.state) {
            launcher.setAdjustmentState(Launcher.AdjustmentState.INCREMENT);
        } else if (gamer2.DPAD_DOWN.state) {
            launcher.setAdjustmentState(Launcher.AdjustmentState.DECREMENT);
        } else {
            launcher.setAdjustmentState(Launcher.AdjustmentState.HOLD);
        }

        // Flicker
        if (gamer1.A.pressed()) {
            if (transfer.getFlickerState() != Transfer.FlickerState.FIRE) {
                transfer.setFlickerState(Transfer.FlickerState.FIRE);
            } else {
                transfer.setFlickerState(Transfer.FlickerState.OFF);
            }
        }
    }
}
