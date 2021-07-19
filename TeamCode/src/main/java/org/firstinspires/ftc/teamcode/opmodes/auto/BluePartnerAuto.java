package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Wobble;
import org.firstinspires.ftc.teamcode.vision.RingPipeline;

import static org.firstinspires.ftc.teamcode.motion.DriveConstants.TRACK_WIDTH;

@Config
@Autonomous
public class BluePartnerAuto extends RobotAuto {
    public static double TURN = -20; //7

    public static final Pose2d startPose = new Pose2d(-63.25, 61.5);
    public static final Vector2d SHOOT_POS = new Vector2d(-2, 63);
    public static final Vector2d INTAKE_POS = new Vector2d(-1, 39);
    public static final Vector2d INTAKE_END_POS = new Vector2d(-8.5, 38);
    public static final Vector2d PARK_POS = new Vector2d(8, 63);
    public static final Vector2d OTHER_PARK_POS = new Vector2d(-2, 52);

    public static DriveConstraints SLOW_CONSTRAINTS = new MecanumConstraints(new DriveConstraints(
            7.5, 7.5, 0.0,
            4.76614956, Math.toRadians(360), 0.0
    ), TRACK_WIDTH);

    Pose2d wobblePos;

    Trajectory toPowershot;
    Trajectory toShoot;
    Trajectory toWobble;
    Trajectory toPark;

    RingPipeline.RingConfiguration ringConfiguration = RingPipeline.RingConfiguration.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        RingPipeline.red = true;
        vision.enable();

        drive.setPoseEstimate(startPose);

        transfer.setPivotState(Transfer.PivotState.UP);
        wobble.setWobbleState(Wobble.WobbleState.OFF);
        wobble.setGrabberState(Wobble.GrabberState.GRABBED);

        while (!opModeIsActive() && !isStopRequested()) {
            String text = vision.getRingPipeline().getCurrentHSV()[0] + ", " + vision.getRingPipeline().getCurrentHSV()[1] + ", " + vision.getRingPipeline().getCurrentHSV()[2];
            telemetry.addData("HSV", text);

            telemetry.addData("Percentage", vision.getRingPipeline().getPercentage());
            telemetry.addData("Ring Config", vision.getRingPipeline().getRingConfiguration());
            telemetry.update();
        }

        waitForStart();

        updateThread.start();

        ringConfiguration = vision.getRingPipeline().getRingConfiguration();
        buildTrajectories(ringConfiguration);
        drive.setPoseEstimate(startPose);

        launcher.setTrajectoryPosition(Launcher.TA_DROP);
        intake.setStickState(Intake.StickState.STOWED);

        transfer.setPivotState(Transfer.PivotState.UP);

        drive.followTrajectory(toShoot);
        drive.turn(Math.toRadians(TURN));
        sleep(600);

        transfer.setSafetyState(Transfer.SafetyState.DISENGAGED);
        transfer.setFlickerState(Transfer.FlickerState.FIRE);
        sleep(1200);

        transfer.setFlickerState(Transfer.FlickerState.OFF);
        launcher.setLauncherState(Launcher.LauncherState.OFF);
        transfer.setFlickerState(Transfer.FlickerState.GO_TO_LIMIT);

        drive.followTrajectory(toWobble);
        sleep(100);
        if (ringConfiguration == RingPipeline.RingConfiguration.ONE) {
            drive.turn(Math.toRadians(0.0));
        } else if (ringConfiguration == RingPipeline.RingConfiguration.NONE || ringConfiguration == RingPipeline.RingConfiguration.FOUR) {
            drive.turn(Math.toRadians(155.0));
        }

        wobble.setWobbleState(Wobble.WobbleState.DOWN);
        sleep(1100);
        wobble.setGrabberState(Wobble.GrabberState.RELEASED);
        sleep(500);

        if (ringConfiguration != RingPipeline.RingConfiguration.NONE) {
            wobble.setWobbleState(Wobble.WobbleState.UP);
        }

        drive.followTrajectory(toPark);



        // go to powershot
//        drive.followTrajectory(toPowershot);
//        launcher.setTrajectoryPosition(Launcher.TA_FLAT+0.03);
//        sleep(400);
//        scorePowershot(0);
//        sleep(800);
//        scorePowershot(RIGHT_ANGLE);
//        sleep(1000);
//        scorePowershot(LEFT_ANGLE);
//        sleep(1000);
    }

    void buildTrajectories(RingPipeline.RingConfiguration ringConfiguration) {
        switch (ringConfiguration) {
            case NULL:
                break;
            case NONE:
                wobblePos = new Pose2d(0, 61.5, 0.0);
                break;
            case ONE:
                wobblePos = new Pose2d(37, 54, Math.toRadians(0.0));
                break;
            case FOUR:
                wobblePos = new Pose2d(48, 61.5, Math.toRadians(0.0));
                break;
        }

        toShoot = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(1, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        launcher.setLauncherState(Launcher.LauncherState.TELE);
                        launcher.setTrajectoryPosition(Launcher.TA_LINE);
                    }
                })
                .lineToConstantHeading(SHOOT_POS)
                .build();

        if (ringConfiguration == RingPipeline.RingConfiguration.ONE) {
            toWobble = drive.trajectoryBuilder(new Pose2d(toShoot.end().vec(), TURN))
                    .splineTo(wobblePos.vec(), 0.0)
                    .addTemporalMarker(0.8, new MarkerCallback() {
                        @Override
                        public void onMarkerReached() {
//                        wobble.setWobbleState(Wobble.WobbleState.DOWN);
                        }
                    })
                    .build();
        } else {
            toWobble = drive.trajectoryBuilder(new Pose2d(toShoot.end().vec(), TURN))
                    .lineTo(wobblePos.vec())
                    .addTemporalMarker(0.8, new MarkerCallback() {
                        @Override
                        public void onMarkerReached() {
//                        wobble.setWobbleState(Wobble.WobbleState.DOWN);
                        }
                    })
                    .build();
        }

//        double parkStartHeading = ringConfiguration == RingPipeline.RingConfiguration.ONE ? Math.toRadians(179.0) : 0.0;
        if (ringConfiguration == RingPipeline.RingConfiguration.NONE) {
            toPark = drive.trajectoryBuilder(new Pose2d(toWobble.end().vec(), Math.toRadians(90)))
                    .lineTo(OTHER_PARK_POS)
                    .build();
        } else {
            toPark = drive.trajectoryBuilder(new Pose2d(toWobble.end().vec(), 0.0))
                    .lineTo(PARK_POS)
                    .build();
        }


        //        toPowershot = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(PS_POS, 0.0))//                .addTemporalMarker(1.5, new MarkerCallback() {
//                    @Override
//                    public void onMarkerReached() {
//                        launcher.setLauncherState(Launcher.LauncherState.WOBBLE);
//                    }
//                })
//                .build();
    }

    void scorePowershot(double targetAngle) throws InterruptedException {
        drive.turn(Math.toRadians((targetAngle)));

        transfer.setSafetyState(Transfer.SafetyState.DISENGAGED);
        transfer.setFlickerState(Transfer.FlickerState.FIRE_ONE);
    }
}
