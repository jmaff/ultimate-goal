package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.vision.RingPipeline;

@Config
@Autonomous
public class RemoteAuto extends RobotOpMode {
    enum State {
        MOVE_TO_SHOOT,
        SHOOT,
        INTAKE_OTHER_RINGS,
        SHOOT_OTHER_RINGS,
        MOVE_FIRST_WOBBLE,
        BACK_TO_WALL,

        IDLE
    }

    public static double TURN = Math.toRadians(-2.1);

    State currentState = State.IDLE;

    Pose2d startPose = new Pose2d(-72.0 + 17.5/2.0, -24-2-3.0/8.0-6-1.0/8.0);
    public static Vector2d SHOOT_POS = new Vector2d(-39.66, -36.29);
    public static Vector2d FIRST_WOBBLE_POS = new Vector2d(15.0, -42.0);

    Trajectory toShootTraj;
    Trajectory firstWobbleTraj;
    Trajectory otherRingIntakeTraj;
    Trajectory otherRingToShootTraj;
    Trajectory backToWallTraj;

    long stateStartTime = 0L;

    RingPipeline.RingConfiguration ringConfiguration = RingPipeline.RingConfiguration.NULL;

    @Override
    public void init() {
        super.init();
        vision.enable();

        drive.setPoseEstimate(startPose);

        toShootTraj = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(SHOOT_POS, TURN))
                .build();

        otherRingIntakeTraj = drive.trajectoryBuilder(toShootTraj.end())
                .lineToLinearHeading(new Pose2d(toShootTraj.end().getX() + 7.5, toShootTraj.end().getY(), 0.0))
                .build();

        otherRingToShootTraj = drive.trajectoryBuilder(otherRingIntakeTraj.end())
                .lineToLinearHeading(new Pose2d(SHOOT_POS, TURN))
                .build();

        firstWobbleTraj = drive.trajectoryBuilder(toShootTraj.end())
                .splineTo(FIRST_WOBBLE_POS, 0.0)
                .build();

        backToWallTraj = drive.trajectoryBuilder(firstWobbleTraj.end(), true)
                .splineTo(new Vector2d(startPose.getX()+1.5, startPose.getY()), Math.toRadians(180.0))
                .build();

        transfer.setPivotState(Transfer.PivotState.UP);
        wobble.setWobbleState(Wobble.WobbleState.OFF);
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
        drive.followTrajectoryAsync(toShootTraj);
    }

    boolean doneTraveling = false;

    @Override
    public void loop() {
        super.loop();

        switch (currentState) {
            case MOVE_TO_SHOOT:
                if (!drive.isBusy()) {
                    setState(State.SHOOT);
                }
                launcher.setLauncherState(Launcher.LauncherState.ON);
                break;
            case SHOOT:
                transfer.setFlickerState(Transfer.FlickerState.FIRE);
                if (getStateElapsedTime() > 1000) {
                    transfer.setFlickerState(Transfer.FlickerState.OFF);
                    launcher.setLauncherState(Launcher.LauncherState.OFF);

                    if (ringConfiguration == RingPipeline.RingConfiguration.ONE || ringConfiguration == RingPipeline.RingConfiguration.FOUR) {
                        // INTAKE OTHER RINGS
                        setState(State.INTAKE_OTHER_RINGS);
                        drive.followTrajectoryAsync(otherRingIntakeTraj);
                        transfer.setFlickerState(Transfer.FlickerState.GO_TO_LIMIT);
                    } else {
                        setState(State.MOVE_FIRST_WOBBLE);
                        drive.followTrajectoryAsync(firstWobbleTraj);
                    }
                }
                break;
            case INTAKE_OTHER_RINGS:
                transfer.setPivotState(Transfer.PivotState.DOWN);
                intake.setState(Intake.IntakeState.IN);
                if (!drive.isBusy()) {
                    setState(State.IDLE);
                }

            case MOVE_FIRST_WOBBLE:
                if (!drive.isBusy() && !doneTraveling) {
                    wobble.setWobbleState(Wobble.WobbleState.DOWN);

                    doneTraveling = true;
                    stateStartTime = System.currentTimeMillis();
                }

                if (doneTraveling) {
                    if (getStateElapsedTime() > 700) {
                        wobble.setWobbleState(Wobble.WobbleState.UP);
                        setState(State.BACK_TO_WALL);
                        drive.followTrajectoryAsync(backToWallTraj);
                    }
                }
                break;
            case BACK_TO_WALL:
                if (!drive.isBusy()) {
                    wobble.setWobbleState(Wobble.WobbleState.GRAB);
                    stateStartTime = System.currentTimeMillis();
                }
                break;
            case IDLE:
                // Do nothing in IDLE
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
        telemetry.update();
    }

    private void setState(State state) {
        currentState = state;
        stateStartTime = System.currentTimeMillis();
    }

    private long getStateElapsedTime() {
        return System.currentTimeMillis() - stateStartTime;
    }
}
