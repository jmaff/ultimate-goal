package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.vision.RingPipeline;

import static org.firstinspires.ftc.teamcode.motion.DriveConstants.TRACK_WIDTH;

@Config
@Autonomous
public class RemoteAuto extends RobotOpMode {
    enum State {
        MOVE_TO_SHOOT,
        SHOOT,
        INTAKE_OTHER_RINGS,
        MOVE_TO_SHOOT_OTHER,
        SHOOT_OTHER,
        MOVE_FIRST_WOBBLE,
        BACK_TO_WALL,
        STRAFE_OVER,
        GRAB_WOBBLE,
        SECOND_WOBBLE,
        PARK,
        POWERSHOT,
        IDLE
    }

    public static double TURN = Math.toRadians(-1.5);
    public static double FORWARD_DIST = 16.25;
    public static double TILE_SIZE_X = 25;
    public static double TILE_SIZE_Y = 19;

    State currentState = State.IDLE;

    public static final Pose2d startPose = new Pose2d(-72.0 + 17.5/2.0, -24-2-3.0/8.0-6-1.0/8.0);
    public static final Vector2d SHOOT_POS = new Vector2d(-39.66, -36.29);
    Vector2d FIRST_WOBBLE_POS = new Vector2d(10.5, -49.0);
    Vector2d TO_WALL_POS = new Vector2d(startPose.getX()+2.0, startPose.getY()+1.5);
    Vector2d SECOND_WOBBLE_POS = new Vector2d(2.5, -42.0);

    Trajectory toShootTraj;
    Trajectory firstWobbleTraj;
    Trajectory otherRingIntakeTraj;
    Trajectory otherRingToShootTraj;
    Trajectory backToWallTraj;
    Trajectory strafeOverTraj;
    Trajectory forwardTraj;
    Trajectory secondWobbleTraj;
    Trajectory parkTraj;
    Trajectory endTraj;

    long stateStartTime = 0L;

    RingPipeline.RingConfiguration ringConfiguration = RingPipeline.RingConfiguration.NULL;

    public static DriveConstraints SLOW_CONSTRAINTS = new MecanumConstraints(new DriveConstraints(
            7.5, 7.5, 0.0,
            4.76614956, Math.toRadians(360), 0.0
    ), TRACK_WIDTH);

    @Override
    public void init() {
        super.init();
        vision.enable();

        drive.setPoseEstimate(startPose);

        toShootTraj = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(SHOOT_POS, TURN))
                .build();

        transfer.setPivotState(Transfer.PivotState.UP);
        wobble.setWobbleState(Wobble.WobbleState.OFF);
        wobble.setGrabberState(Wobble.GrabberState.GRABBED);
    }

    @Override
    public void init_loop() {
        super.init_loop();
        String text = vision.getRingPipeline().getCurrentHSV()[0] + ", " + vision.getRingPipeline().getCurrentHSV()[1] + ", " + vision.getRingPipeline().getCurrentHSV()[2];
        telemetry.addData("HSV", text);

        telemetry.addData("Percentage", vision.getRingPipeline().getPercentage());

        String ring = "";
        switch (vision.getRingPipeline().getRingConfiguration()) {
            case NULL:
                ring = "NULL";
                break;
            case NONE:
                ring = "NONE";
                break;
            case ONE:
                ring = "ONE";
                break;
            case FOUR:
                ring = "FOUR";
                break;
        }
        telemetry.addData("Ring Config", ring);
    }

    @Override
    public void start() {
        super.start();
        ringConfiguration = vision.getRingPipeline().getRingConfiguration();
        currentState = State.MOVE_TO_SHOOT;
        drive.setPoseEstimate(startPose);
        drive.followTrajectoryAsync(toShootTraj);

        launcher.setTrajectoryPosition(Launcher.TA_DROP);
        buildTrajectories(ringConfiguration);
        setState(State.MOVE_TO_SHOOT);
        intake.setStickState(Intake.StickState.STOWED);

    }

    boolean doneTraveling = false;
    boolean movedFlicker = false;
    boolean waiting = false;

    int shots = 0;

    @Override
    public void loop() {
        super.loop();

        switch (currentState) {
            // AUTO START
            case MOVE_TO_SHOOT:
                if (getStateElapsedTime() > 700) {
                    launcher.setTrajectoryPosition(Launcher.TA_FLAT);
                }
                if (!drive.isBusy()) {
                    setState(State.SHOOT);
                }
                launcher.setLauncherState(Launcher.LauncherState.WOBBLE);
                break;
            case SHOOT:
                if (!doneTraveling) {
                    if (shots == 4) { //getStateElapsedTime() > 1000
                        transfer.setFlickerState(Transfer.FlickerState.OFF);
                        launcher.setLauncherState(Launcher.LauncherState.OFF);
                        doneTraveling = true;
                        stateStartTime = System.currentTimeMillis();
                    } else {
                        if (transfer.getFlickerState() != Transfer.FlickerState.FIRE_ONE || getStateElapsedTime() > 400) {
                            if (!waiting) {
                                waiting = true;
                                stateStartTime = System.currentTimeMillis();
                            } else {
                                waiting = false;
                                shots++;
                                transfer.setFlickerState(Transfer.FlickerState.FIRE_ONE);
                            }

                        }
//                        transfer.setFlickerState(Transfer.FlickerState.FIRE);
                    }
                }

                if (doneTraveling) {
                    if ((ringConfiguration == RingPipeline.RingConfiguration.ONE || ringConfiguration == RingPipeline.RingConfiguration.FOUR)) {
                        if (!movedFlicker) {
                            transfer.setFlickerState(Transfer.FlickerState.GO_TO_LIMIT);
                            movedFlicker = true;
                        }
                        // INTAKE OTHER RINGS
                        if (getStateElapsedTime() > 700) {
                            setState(State.INTAKE_OTHER_RINGS);
                            drive.followTrajectoryAsync(otherRingIntakeTraj);
                        }

                    } else {
                        setState(State.MOVE_FIRST_WOBBLE);
                        drive.followTrajectoryAsync(firstWobbleTraj);
                        if (ringConfiguration == RingPipeline.RingConfiguration.NONE) {
                            wobble.setWobbleState(Wobble.WobbleState.DOWN);
                        }
                    }
                }
                break;
            case INTAKE_OTHER_RINGS:
                transfer.setSafetyState(Transfer.SafetyState.ENGAGED);
                if (drive.isBusy()) {
                    transfer.setPivotState(Transfer.PivotState.DOWN);
                    intake.setState(Intake.IntakeState.IN);
                    stateStartTime = System.currentTimeMillis();
                }
                if (!drive.isBusy() && !doneTraveling) {
                    if (getStateElapsedTime() > 700) {
                        doneTraveling = true;
                        stateStartTime = System.currentTimeMillis();
                        transfer.setPivotState(Transfer.PivotState.UP);

                        transfer.setFlickerState(Transfer.FlickerState.OFF);
                    }
                }
                if (doneTraveling && getStateElapsedTime() > 700) {
                    intake.setState(Intake.IntakeState.OFF);
                    launcher.setLauncherState(Launcher.LauncherState.ON);
                    drive.followTrajectoryAsync(otherRingToShootTraj);
                    setState(State.MOVE_TO_SHOOT_OTHER);
                }
                break;
            case MOVE_TO_SHOOT_OTHER:
                if (!drive.isBusy()) {
                    setState(State.SHOOT_OTHER);
                    shots = 0;
                }
                break;
            case SHOOT_OTHER:
                transfer.setSafetyState(Transfer.SafetyState.DISENGAGED);
                if (!doneTraveling) {
                    if (shots == 4) { //getStateElapsedTime() > 1000
                        transfer.setFlickerState(Transfer.FlickerState.OFF);
                        launcher.setLauncherState(Launcher.LauncherState.OFF);
                        doneTraveling = true;
                        stateStartTime = System.currentTimeMillis();
                    } else {
                        if (transfer.getFlickerState() != Transfer.FlickerState.FIRE_ONE || getStateElapsedTime() > 400) {
                            if (!waiting) {
                                waiting = true;
                                stateStartTime = System.currentTimeMillis();
                            } else {
                                waiting = false;
                                shots++;
                                transfer.setFlickerState(Transfer.FlickerState.FIRE_ONE);
                            }

                        }
//                        transfer.setFlickerState(Transfer.FlickerState.FIRE);
                    }
                }

                if (doneTraveling) {
                    setState(State.MOVE_FIRST_WOBBLE);
                    drive.followTrajectoryAsync(firstWobbleTraj);
                    intake.setState(Intake.IntakeState.IN);
                    transfer.setPivotState(Transfer.PivotState.DOWN);
                    wobble.setWobbleState(Wobble.WobbleState.DOWN);
                }
                break;

            case MOVE_FIRST_WOBBLE:
                transfer.setSafetyState(Transfer.SafetyState.ENGAGED);
                if (!drive.isBusy() && !doneTraveling) {
                    doneTraveling = true;
                    stateStartTime = System.currentTimeMillis();
                }

                if (doneTraveling) {
                    wobble.setGrabberState(Wobble.GrabberState.RELEASED);

                    if (getStateElapsedTime() > 600) { //1600
                        wobble.setWobbleState(Wobble.WobbleState.UP);
                        setState(State.BACK_TO_WALL);
                        drive.followTrajectoryAsync(backToWallTraj);
                        intake.setState(Intake.IntakeState.OFF);
                    }
                }
                break;
            case BACK_TO_WALL:
                if (!drive.isBusy() && !doneTraveling) {
                    doneTraveling = true;
                    stateStartTime = System.currentTimeMillis();
                    wobble.setWobbleState(Wobble.WobbleState.GRAB);
                }
                if (doneTraveling && getStateElapsedTime() > 600) { //1000
                    drive.followTrajectoryAsync(strafeOverTraj);
                    setState(State.STRAFE_OVER);
                }
                break;
            case STRAFE_OVER:
                if (!drive.isBusy()) {
                    drive.followTrajectoryAsync(forwardTraj);
                    setState(State.GRAB_WOBBLE);
                }
                break;
            case GRAB_WOBBLE:
                if (!drive.isBusy() && !doneTraveling) {
                    doneTraveling = true;
                    stateStartTime = System.currentTimeMillis();
                    wobble.setGrabberState(Wobble.GrabberState.GRABBED);
                }
                if (doneTraveling && getStateElapsedTime() > 600) {
                    drive.followTrajectoryAsync(secondWobbleTraj);
                    setState(State.SECOND_WOBBLE);
                    transfer.setPivotState(Transfer.PivotState.UP);
                }
                break;
            case SECOND_WOBBLE:
                if (!drive.isBusy() && !doneTraveling) {
                    doneTraveling = true;
                    stateStartTime = System.currentTimeMillis();
                }

                if (doneTraveling) {
                    wobble.setGrabberState(Wobble.GrabberState.RELEASED);
                    if (getStateElapsedTime() > 900) {
                        wobble.setWobbleState(Wobble.WobbleState.UP);
                        if (ringConfiguration == RingPipeline.RingConfiguration.NONE) {
                            setState(State.IDLE);
                        } else {
                            drive.followTrajectoryAsync(parkTraj);
                            setState(State.PARK);
                        }
                    }
                }
                break;
            case PARK:
                if (ringConfiguration == RingPipeline.RingConfiguration.FOUR) {
                    launcher.setLauncherState(Launcher.LauncherState.ON);
                }

                launcher.setTrajectoryPosition(Launcher.TA_LINE);
                if (!drive.isBusy() && ringConfiguration == RingPipeline.RingConfiguration.FOUR) {
                    setState(State.POWERSHOT);
                }
                break;
            case POWERSHOT:
                transfer.setSafetyState(Transfer.SafetyState.DISENGAGED);
                transfer.setFlickerState(Transfer.FlickerState.FIRE);

                if (getStateElapsedTime() > 1000 && !doneTraveling) {
                    drive.followTrajectoryAsync(endTraj);
                    launcher.setLauncherState(Launcher.LauncherState.OFF);
                    doneTraveling = true;
                }
                break;
            case IDLE:
                if (getStateElapsedTime() > 700) {
                    wobble.setWobbleState(Wobble.WobbleState.UP);
                }
                break;
        }

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Continually write pose to `PoseStorage`
//            PoseStorage.currentPose = poseEstimate;

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("flicker", transfer.getFlickerState());
        telemetry.update();
    }

    void buildTrajectories(RingPipeline.RingConfiguration ringConfiguration) {
        double INTAKE_DISTANCE =  ringConfiguration == RingPipeline.RingConfiguration.ONE ? 7.5 : 9;

        otherRingIntakeTraj = (new TrajectoryBuilder(toShootTraj.end(), SLOW_CONSTRAINTS))
                .lineToLinearHeading(new Pose2d(toShootTraj.end().getX() + INTAKE_DISTANCE, toShootTraj.end().getY(), 0.0))
                .build();

        otherRingToShootTraj = drive.trajectoryBuilder(otherRingIntakeTraj.end())
                .lineToLinearHeading(new Pose2d(SHOOT_POS, 0.0))
                .build();

        if (ringConfiguration == RingPipeline.RingConfiguration.ONE) {
            FIRST_WOBBLE_POS = new Vector2d(39, -28.0);
            SECOND_WOBBLE_POS = new Vector2d(29.5, -20);
        } else if (ringConfiguration == RingPipeline.RingConfiguration.FOUR) {
            FIRST_WOBBLE_POS = new Vector2d(54, -51.3);
            SECOND_WOBBLE_POS = new Vector2d(56, -43);
        } else if (ringConfiguration == RingPipeline.RingConfiguration.NONE) {
            FIRST_WOBBLE_POS = new Vector2d(13.0, -51.0);
            SECOND_WOBBLE_POS = new Vector2d(7.5, -45.0);
        }

        if (ringConfiguration != RingPipeline.RingConfiguration.FOUR) {
            firstWobbleTraj = drive.trajectoryBuilder(toShootTraj.end())
                    .splineTo(FIRST_WOBBLE_POS, 0.0)
                    .build();
        } else {
            firstWobbleTraj = drive.trajectoryBuilder(toShootTraj.end())
                    .forward(6)
                    .splineTo(FIRST_WOBBLE_POS, 0.0)
                    .build();
        }

        backToWallTraj = drive.trajectoryBuilder(firstWobbleTraj.end(), true)
                .splineTo(TO_WALL_POS, Math.toRadians(180))
                .build();

        if (ringConfiguration == RingPipeline.RingConfiguration.NONE) {
            strafeOverTraj = drive.trajectoryBuilder(backToWallTraj.end())
                    .strafeRight(3.2)
                    .build();
        } else if (ringConfiguration == RingPipeline.RingConfiguration.ONE){
            strafeOverTraj = drive.trajectoryBuilder(backToWallTraj.end())
                    .strafeRight(3)
                    .build();
        } else {
            strafeOverTraj = drive.trajectoryBuilder(backToWallTraj.end())
                    .strafeRight(1)
                    .build();
        }

        if (ringConfiguration != RingPipeline.RingConfiguration.NONE) {
            FORWARD_DIST = 17.2;
        }

        forwardTraj = drive.trajectoryBuilder(strafeOverTraj.end())
                .forward(FORWARD_DIST)
                .addDisplacementMarker(new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobble.setGrabberState(Wobble.GrabberState.GRABBED);
                    }
                })
                .build();

        secondWobbleTraj = drive.trajectoryBuilder(forwardTraj.end())
                .splineTo(SECOND_WOBBLE_POS, 0.0)
                .build();


        if (ringConfiguration == RingPipeline.RingConfiguration.ONE) {
            parkTraj = drive.trajectoryBuilder(secondWobbleTraj.end())
                    .back(30)
                    .build();
        } else {
            parkTraj = drive.trajectoryBuilder(secondWobbleTraj.end())
                    .lineToLinearHeading(new Pose2d(-5, -32, 0)) // toRadians(15)
                    .build();
        }

        endTraj = drive.trajectoryBuilder(parkTraj.end())
                .forward(10)
                .build();
    }

    private void setState(State state) {
        doneTraveling = false;
        currentState = state;
        stateStartTime = System.currentTimeMillis();
    }

    private long getStateElapsedTime() {
        return System.currentTimeMillis() - stateStartTime;
    }
}
