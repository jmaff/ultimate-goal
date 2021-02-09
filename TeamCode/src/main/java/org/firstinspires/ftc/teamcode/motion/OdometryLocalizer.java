package org.firstinspires.ftc.teamcode.motion;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

@Config
public class OdometryLocalizer extends CustomThreeWheelTrackingLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 1; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 15; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 1; // in; offset of the lateral wheel

    // 118.9072, 118.8373, 118.7849
//    public static double X_MULTIPLIER = (118.0 * 3) / (118.9072 + 118.8373 + 118.7849);
    // 119.0795, 119.1408, 119.1883
//    public static double Y_MULTIPLIER = (118.0 * 3) / (119.0795 + 119.1408 + 119.1883);

    public static double X_MULTIPLIER = 0.9933036198;
    public static double Y_MULTIPLIER = 0.9917013429;

    private Encoder leftEncoder, rightEncoder, lateralEncoder;

    public static double EFFECTIVE_LATERAL_DIST = 13.49427711;
//public static double EFFECTIVE_LATERAL_DIST = 15.261066;
    public static double FORWARD_Y = 6 + (7.0/8.0);
    public static double FORWARD_X = 1.125;
    public static double LATERAL_Y = 7 + (5.0/8.0);
    public static double LATERAL_X = 1.25;

    private Drivetrain drive;

    public OdometryLocalizer(HardwareMap hardwareMap, Drivetrain drive) {
//        super(Arrays.asList(
//                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
//                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
//                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // lateral
//        ));
        super(Arrays.asList(
                new Pose2d(LATERAL_X, EFFECTIVE_LATERAL_DIST/2, 0), // left
                new Pose2d(LATERAL_X, -EFFECTIVE_LATERAL_DIST/2, 0), // right
                new Pose2d(FORWARD_X, FORWARD_Y, Math.toRadians(90)) // lateral
        ));
        this.drive = drive;

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "D.BL"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "D.FL"));
        lateralEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "D.FR"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        lateralEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition() * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCurrentPosition() * X_MULTIPLIER),
                encoderTicksToInches(lateralEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity() * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity() * X_MULTIPLIER),
                encoderTicksToInches(lateralEncoder.getCorrectedVelocity() * Y_MULTIPLIER)
        );
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }
}
