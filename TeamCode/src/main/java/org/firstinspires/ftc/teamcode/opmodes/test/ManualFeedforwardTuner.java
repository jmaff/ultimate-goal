package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.motion.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Objects;

import static org.firstinspires.ftc.teamcode.motion.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.motion.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.motion.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.motion.DriveConstants.kV;

@Disabled
@Config
@Autonomous(group = "drive")
public class ManualFeedforwardTuner extends LinearOpMode {
    public static double DISTANCE = 72; // in

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private Drivetrain drive;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private Mode mode;

    private static MotionProfile generateProfile(boolean movingForward) {
        MotionState start = new MotionState(movingForward ? 0 : DISTANCE, 0, 0, 0);
        MotionState goal = new MotionState(movingForward ? DISTANCE : 0, 0, 0, 0);
        return MotionProfileGenerator.generateSimpleMotionProfile(start, goal,
                DriveConstants.BASE_CONSTRAINTS.maxVel,
                DriveConstants.BASE_CONSTRAINTS.maxAccel,
                DriveConstants.BASE_CONSTRAINTS.maxJerk);
    }

    @Override
    public void runOpMode() {
        if (RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new Drivetrain(hardwareMap);

        mode = Mode.TUNING_MODE;

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        boolean movingForwards = true;
        MotionProfile activeProfile = generateProfile(true);
        double profileStart = clock.seconds();


        while (!isStopRequested()) {
            telemetry.addData("mode", mode);

            switch (mode) {
                case TUNING_MODE:
                    if (gamepad1.x) {
                        mode = Mode.DRIVER_MODE;
                    }

                    // calculate and set the motor power
                    double profileTime = clock.seconds() - profileStart;

                    if (profileTime > activeProfile.duration()) {
                        // generate a new profile
                        movingForwards = !movingForwards;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    MotionState motionState = activeProfile.get(profileTime);
                    double targetPower = Kinematics.calculateMotorFeedforward(motionState.getV(), motionState.getA(), kV, kA, kStatic);

                    drive.setDrivePower(new Pose2d(targetPower, 0, 0));
                    drive.updatePoseEstimate();

                    Pose2d poseVelo = Objects.requireNonNull(drive.getPoseVelocity(), "poseVelocity() must not be null. Ensure that the getWheelVelocities() method has been overridden in your localizer.");
                    double currentVelo = poseVelo.getX();

                    // update telemetry
                    telemetry.addData("targetVelocity", motionState.getV());
                    telemetry.addData("poseVelocity", currentVelo);
                    telemetry.addData("error", currentVelo - motionState.getV());
                    break;
                case DRIVER_MODE:
                    if (gamepad1.a) {
                        mode = Mode.TUNING_MODE;
                        movingForwards = true;
                        activeProfile = generateProfile(movingForwards);
                        profileStart = clock.seconds();
                    }

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                    break;
            }

            telemetry.update();
        }
    }
}
