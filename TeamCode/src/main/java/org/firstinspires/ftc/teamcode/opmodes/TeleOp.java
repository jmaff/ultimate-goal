package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Wobble;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends RobotOpMode {
    public static long TIME_TO_DISENGAGE = 500;
    enum TeleOpState {
        DRIVER_CONTROLLED,
        JAM_RECOVERY,
        POWERSHOT
    }

    Trajectory powerShotTraj;
    static double POWERSHOT_STRAFE = 17.9;

    Runnable buildTrajectoriesRunnable = new Runnable() {
        @Override
        public void run() {
            powerShotTraj = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .strafeRight(POWERSHOT_STRAFE)
                    .build();
        }
    };

    int powerShotsDone = 0;
    boolean shot = false;
    boolean shooting = false;
    long shootStartTime = 0;

    long fireStartTime = 0;
    boolean teleOpShooting = false;

    Thread buildThread = new Thread(buildTrajectoriesRunnable);

    TeleOpState state = TeleOpState.DRIVER_CONTROLLED;

    @Override
    public void init() {
        super.init();
        drive.setPoseEstimate(new Pose2d(-72.0 + 17.5/2.0, -24-2-3.0/8.0-6-1.0/8.0));
        wobble.setWobbleState(Wobble.WobbleState.MANUAL);
        launcher.setTrajectoryPosition(Launcher.TA_LINE);
        intake.setStickState(Intake.StickState.DOWN);
        transfer.setSafetyState(Transfer.SafetyState.ENGAGED);
        powerShotsDone = 0;
        POWERSHOT_STRAFE = 17.7;
    }

    @Override
    public void loop() {
        super.loop();

        switch (state) {
            case DRIVER_CONTROLLED:
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
                        launcher.setLauncherState(Launcher.LauncherState.OFF);
                        transfer.setSafetyState(Transfer.SafetyState.ENGAGED);
                        intake.setStickState(Intake.StickState.DOWN);
                        teleOpShooting = false;
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
                    intake.setState(Intake.IntakeState.OFF);
                    intake.setStickState(Intake.StickState.RAISED);
                }
                if (gamer2.LEFT_BUMPER.pressed()) {
                    transfer.setPivotState(Transfer.PivotState.DOWN);
                    transfer.setFlickerState(Transfer.FlickerState.OFF);
                    intake.setStickState(Intake.StickState.DOWN);
                }

                // Launcher
                if (gamer2.B.pressed()) {
                    if (launcher.getLauncherState() != Launcher.LauncherState.TELE) {
                        launcher.setLauncherState(Launcher.LauncherState.TELE);
                    } else {
                        launcher.setLauncherState(Launcher.LauncherState.OFF);
                    }
                }

                if (gamer2.X.pressed()) {
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

                if (gamer2.DPAD_RIGHT.state) {
                    launcher.setTrajectoryPosition(Launcher.TA_LINE);
                }

                if (gamer2.LEFT_JOYSTICK_PUSH.state) {
                    transfer.setFlickerState(Transfer.FlickerState.MANUAL);
                    transfer.flicker.setPower(0.5);
                } else if (gamer2.RIGHT_JOYSTICK_PUSH.state) {
                    transfer.setFlickerState(Transfer.FlickerState.MANUAL);
                    transfer.flicker.setPower(-0.5);
                } else if (transfer.getFlickerState() == Transfer.FlickerState.MANUAL) {
                    transfer.setFlickerState(Transfer.FlickerState.OFF);
                }

                // Wobble (manual)
                if (gamer2.DPAD_UP.state) {
                    wobble.setPower(-Wobble.UP_POWER);
                } else if (gamer2.DPAD_DOWN.state) {
                    wobble.setPower(Wobble.DOWN_POWER);
                } else {
                    wobble.setPower(0.0);
                }

                if (gamer1.X.state) {
                    wobble.setGrabberState(Wobble.GrabberState.GRABBED);
                } else if (gamer1.B.state) {
                    wobble.setGrabberState(Wobble.GrabberState.RELEASED);
                }

                // Flicker
                if (gamer1.A.pressed()) {
                    if (transfer.getFlickerState() != Transfer.FlickerState.TELE) {
                        intake.setStickState(Intake.StickState.DOWN);
                        teleOpShooting = true;
                        fireStartTime = System.currentTimeMillis();
                    } else {
                        transfer.setFlickerState(Transfer.FlickerState.OFF);
                        teleOpShooting = false;
                    }
                }

                if (teleOpShooting && fireStartTime > TIME_TO_DISENGAGE) {
                    transfer.setFlickerState(Transfer.FlickerState.TELE);
                }

                if (transfer.getFlickerState() == Transfer.FlickerState.TELE && fireStartTime > TIME_TO_DISENGAGE) {
                    transfer.setSafetyState(Transfer.SafetyState.DISENGAGED);
                }

                if (gamer1.DPAD_RIGHT.state && gamer1.Y.state) {
                    if (powerShotTraj == null && !buildThread.isAlive()) {
                        buildThread.start();
                    } else if (powerShotTraj != null) {
                        state = TeleOpState.POWERSHOT;
                        transfer.setPivotState(Transfer.PivotState.UP);
                        launcher.setTrajectoryPosition(Launcher.TA_FLAT);
                        drive.followTrajectoryAsync(powerShotTraj);
                        launcher.setLauncherState(Launcher.LauncherState.WOBBLE);
                    }
                }

                if (gamer2.BACK.state) {
                    transfer.setSafetyState(Transfer.SafetyState.DISENGAGED);
                }

                if (gamer1.RIGHT_JOYSTICK_PUSH.state) {
                    transfer.setFlickerState(Transfer.FlickerState.GO_TO_LIMIT);
                }

                if (gamer1.DPAD_UP.pressed()) {
                    intake.setStickState(Intake.StickState.STOWED);
                }

                if (gamer1.DPAD_DOWN.pressed()) {
                    transfer.setFlickerState(Transfer.FlickerState.FIRE_ONE);
                }

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.addData("Wobble Counts", wobble.getPosition());
                telemetry.addData("Position", launcher.getTrajectoryPosition());

                break;
            case JAM_RECOVERY:
                break;
            case POWERSHOT:
                telemetry.addData("STATUS", "**AUTO POWERSHOT**");
                POWERSHOT_STRAFE = 9;
                if (!drive.isBusy()) {
                    transfer.setSafetyState(Transfer.SafetyState.DISENGAGED);
                    if (powerShotsDone < 3) {
                        if (!shot && !shooting) {
                            shootStartTime = System.currentTimeMillis();
                            shooting = true;
                            launcher.setLauncherState(Launcher.LauncherState.WOBBLE);
                            transfer.setFlickerState(Transfer.FlickerState.FIRE_ONE);
                        } else if (!shot) {
                            long elapsed = System.currentTimeMillis() - shootStartTime;
                            if (transfer.getFlickerState() == Transfer.FlickerState.OFF) {
                                shot = true;
                                shooting = false;
                                powerShotsDone++;
                                buildThread.start();
                            }
                        } else {
                            if (!buildThread.isAlive()) {
                                shot = false;
                                shooting = false;
                                drive.followTrajectoryAsync(powerShotTraj);
                            }
                        }
                    } else {
                        state = TeleOpState.DRIVER_CONTROLLED;
                        powerShotsDone = 0;
                    }
                }
                break;
        }


        telemetry.update();
    }
}
