package org.firstinspires.ftc.teamcode.motion

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.Encoder

@Config
class OdometryLocalizer(hardwareMap: HardwareMap) : ThreeTrackingWheelLocalizer(listOf(
        Pose2d(0.0, PARALLEL_DISTANCE / 2, 0.0),  // left
        Pose2d(0.0, -PARALLEL_DISTANCE / 2, 0.0),  // right
        Pose2d(FORWARD_OFFSET, -PARALLEL_DISTANCE / 2, Math.toRadians(90.0)) // perp
)) {
    private val leftEncoder: Encoder
    private val rightEncoder: Encoder
    private val frontEncoder: Encoder
    override fun getWheelPositions(): List<Double> {
        return listOf(
                encoderTicksToInches(leftEncoder.currentPosition.toDouble() * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.currentPosition.toDouble() * X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.currentPosition.toDouble() * Y_MULTIPLIER)
        )
    }

    override fun getWheelVelocities(): List<Double> {
        return listOf(
                encoderTicksToInches(leftEncoder.correctedVelocity),
                encoderTicksToInches(rightEncoder.correctedVelocity),
                encoderTicksToInches(frontEncoder.correctedVelocity)
        )
    }

    companion object {
        @JvmField var TICKS_PER_REV = 8192.0
        var WHEEL_RADIUS = 1.0 // in
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        @JvmField var PARALLEL_DISTANCE = 15.0 // in; distance between the left and right wheels
        @JvmField var FORWARD_OFFSET = 1.0 // in; offset of the lateral wheel
        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
        // 118.9072, 118.8373, 118.7849
        @JvmField var X_MULTIPLIER = (118.0 * 3) / (118.9072 + 118.8373 + 118.7849)
        // 119.0795, 119.1408, 119.1883
        @JvmField var Y_MULTIPLIER = (118.0 * 3) / (119.0795 + 119.1408 + 119.1883)
    }

    init {
        leftEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "D.LF"))
        rightEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "D.RF"))
        frontEncoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "D.RB"))

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder.direction = Encoder.Direction.REVERSE
        rightEncoder.direction = Encoder.Direction.REVERSE
    }
}