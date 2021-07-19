package org.firstinspires.ftc.teamcode.opmodes.auto;

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

@Autonomous
public class BlueAuto extends RobotAuto {
    public static double TURN = 0; //7

    public static final Pose2d startPose = new Pose2d(-63.25, 15.5);
//    public static final Vector2d PS_POS = new Vector2d(-2, 12);
    public static final Vector2d SHOOT_POS = new Vector2d(-3, 34);
    public static final Vector2d INTAKE_POS = new Vector2d(-1, 34);
    public static final Vector2d INTAKE_END_POS = new Vector2d(-9, 34);
    public static final Vector2d PARK_POS = new Vector2d(10, 10);

    public static final double RIGHT_ANGLE = -9.5;
    public static final double LEFT_ANGLE = 12;

    public static DriveConstraints SLOW_CONSTRAINTS = new MecanumConstraints(new DriveConstraints(
            7.5, 7.5, 0.0,
            4.76614956, Math.toRadians(360), 0.0
    ), TRACK_WIDTH);

    Pose2d wobblePos;

    Trajectory toPowershot;
    Trajectory toShoot;
    Trajectory toWobble;
    Trajectory toIntake;
    Trajectory toIntakeEnd;
    Trajectory toShootOther;
    Trajectory toPark;

    RingPipeline.RingConfiguration ringConfiguration = RingPipeline.RingConfiguration.NULL;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        RingPipeline.red = false;
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

        wobble.setWobbleState(Wobble.WobbleState.DOWN);
        drive.followTrajectory(toWobble);
        wobble.setGrabberState(Wobble.GrabberState.RELEASED);
        sleep(500);

        wobble.setWobbleState(Wobble.WobbleState.UP);

        if (ringConfiguration == RingPipeline.RingConfiguration.ONE || ringConfiguration == RingPipeline.RingConfiguration.FOUR) {
            transfer.setSafetyState(Transfer.SafetyState.ENGAGED);
            transfer.setPivotState(Transfer.PivotState.DOWN);
            intake.setState(Intake.IntakeState.IN);

            drive.followTrajectory(toIntake);
            sleep(100);
            drive.followTrajectory(toIntakeEnd);
            sleep(800);
            intake.setState(Intake.IntakeState.OUT);
            sleep(300);
            drive.turn(Math.toRadians(180.0));
            sleep(100);
            intake.setState(Intake.IntakeState.OFF);
            transfer.setPivotState(Transfer.PivotState.UP);
            sleep(100);
            drive.followTrajectory(toShootOther);
            sleep(300);
            drive.turn(Math.toRadians(1));
            sleep(600);

            transfer.setSafetyState(Transfer.SafetyState.DISENGAGED);
            transfer.setFlickerState(Transfer.FlickerState.FIRE);
            sleep(800);
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
                wobblePos = new Pose2d(14, 40, Math.toRadians(180));
                break;
            case ONE:
                wobblePos = new Pose2d(40, 15, Math.toRadians(180));
                break;
            case FOUR:
                wobblePos = new Pose2d(60, 40, Math.toRadians(180));
                break;
        }

        toShoot = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-20, 16), 0.0)
                .addTemporalMarker(1, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        launcher.setLauncherState(Launcher.LauncherState.ON);
                        launcher.setTrajectoryPosition(Launcher.TA_LINE);
                    }
                })
                .splineToConstantHeading(SHOOT_POS, 0.0)
                .build();

        toWobble = drive.trajectoryBuilder(new Pose2d(toShoot.end().vec(), TURN))
                .lineToLinearHeading(wobblePos)
                .addTemporalMarker(0.8, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        wobble.setWobbleState(Wobble.WobbleState.DOWN);
                    }
                })
                .build();

        toIntake = drive.trajectoryBuilder(toWobble.end())
                .splineTo(INTAKE_POS, Math.toRadians(180))
                .build();

        toIntakeEnd = drive.trajectoryBuilder(toIntake.end())
                .lineTo(INTAKE_END_POS, SLOW_CONSTRAINTS)
                .build();


        toShootOther = drive.trajectoryBuilder(new Pose2d(toIntake.end().vec(), 0.0))
                .lineToConstantHeading(new Vector2d(SHOOT_POS.getX()-3, SHOOT_POS.getY()))
                .addTemporalMarker(1, new MarkerCallback() {
                    @Override
                    public void onMarkerReached() {
                        launcher.setLauncherState(Launcher.LauncherState.TELE);
                        launcher.setTrajectoryPosition(Launcher.TA_LINE);
                    }
                })
                .build();

        if (ringConfiguration == RingPipeline.RingConfiguration.ONE || ringConfiguration == RingPipeline.RingConfiguration.FOUR) {
            toPark = drive.trajectoryBuilder(toShootOther.end())
                    .lineToConstantHeading(PARK_POS)
                    .build();
        } else {
            toPark = drive.trajectoryBuilder(toWobble.end())
                    .lineToConstantHeading(PARK_POS)
                    .build();
        }

        //        toPowershot = drive.trajectoryBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(PS_POS, 0.0))
//                .addTemporalMarker(1.5, new MarkerCallback() {
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
