package org.firstinspires.ftc.teamcode.motion

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints

object DriveConstants {
    /*
     * These are motor constants that should be listed online for your motors.
     */
    val TICKS_PER_REV = 537.6
    @JvmField val MAX_RPM = 340.0

    val X_DIM = 17.75
    val Y_DIM = 17.5

    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */
    val RUN_USING_ENCODER = false
    val MOTOR_VELO_PID: PIDCoefficients? = null

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    var WHEEL_RADIUS = 2.0
    var GEAR_RATIO = 1.0 // output (wheel) speed / input (motor) speed
    @JvmField var TRACK_WIDTH = 15.0

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    var kV = 1.0 / rpmToVelocity(DriveConstants.MAX_RPM)
    // /rpmToVelocity(MAX_RPM)
    var kA = 0.0
    var kStatic = 0.0

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling).
     */
    var BASE_CONSTRAINTS = DriveConstraints(
            90.0, 90.0, 0.0,
            Math.toRadians(360.0), Math.toRadians(360.0), 0.0
    )

    fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_RADIUS * 2.0 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }

    @JvmStatic
    fun rpmToVelocity(rpm: Double): Double {
        return rpm * GEAR_RATIO * 2.0 * Math.PI * WHEEL_RADIUS / 60.0
    }

    fun getMotorVelocityF(): Double {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (DriveConstants.MAX_RPM * DriveConstants.TICKS_PER_REV)
    }
}