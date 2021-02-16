package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Wobble;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends RobotOpMode {

    @Override
    public void init() {
        super.init();
        drive.setPoseEstimate(new Pose2d(-72.0 + 17.5/2.0, -24-2-3.0/8.0-6-1.0/8.0));
        wobble.setWobbleState(Wobble.WobbleState.MANUAL);
    }

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
        if (gamer2.B.pressed()) {
            if (launcher.getLauncherState() != Launcher.LauncherState.ON) {
                launcher.setLauncherState(Launcher.LauncherState.ON);
            } else {
                launcher.setLauncherState(Launcher.LauncherState.OFF);
            }
        }

        // Trajectory Adjust
        if (gamer2.Y.state) {
            launcher.setAdjustmentState(Launcher.AdjustmentState.INCREMENT);
        } else if (gamer2.A.state) {
            launcher.setAdjustmentState(Launcher.AdjustmentState.DECREMENT);
        } else {
            launcher.setAdjustmentState(Launcher.AdjustmentState.HOLD);
        }

        // Wobble (auto)
        if (gamer2.DPAD_UP.pressed()) {
            if (wobble.getWobbleState() == Wobble.WobbleState.GRAB) {
                wobble.setWobbleState(Wobble.WobbleState.DOWN);
            } else {
                wobble.setWobbleState(Wobble.WobbleState.UP);
            }
        } else if (gamer2.DPAD_DOWN.pressed()) {
            if (wobble.getWobbleState() == Wobble.WobbleState.UP) {
                wobble.setWobbleState(Wobble.WobbleState.DOWN);
            } else {
                wobble.setWobbleState(Wobble.WobbleState.GRAB);
            }
        }

        if (gamer2.LEFT_JOYSTICK_PUSH.state) {
            transfer.setFlickerState(Transfer.FlickerState.GO_TO_LIMIT);
        }

        // Wobble (manual)
//        if (gamer2.DPAD_UP.state) {
//            wobble.setPower(-Wobble.UP_POWER);
//        } else if (gamer2.DPAD_DOWN.state) {
//            wobble.setPower(Wobble.DOWN_POWER);
//        } else {
//            wobble.setPower(0.0);
//        }

        if (gamer1.X.state) {
            wobble.setGrabberState(Wobble.GrabberState.GRABBED);
        } else if (gamer1.B.state) {
            wobble.setGrabberState(Wobble.GrabberState.RELEASED);
        }

        // Flicker
        if (gamer1.A.pressed()) {
            if (transfer.getFlickerState() != Transfer.FlickerState.FIRE) {
                transfer.setFlickerState(Transfer.FlickerState.FIRE);
            } else {
                transfer.setFlickerState(Transfer.FlickerState.OFF);
            }
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Wobble Counts", wobble.getPosition());
        telemetry.update();
    }
}
