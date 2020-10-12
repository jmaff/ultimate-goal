package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain


@TeleOp(group = "drive")
class LocalizationTest : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = Drivetrain(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        waitForStart()
        while (!isStopRequested) {
            drive.setWeightedDrivePower(
                    Pose2d(
                            (-gamepad1.left_stick_y).toDouble(),
                            (-gamepad1.left_stick_x).toDouble(),
                            (-gamepad1.right_stick_x).toDouble()
                    )
            )
            drive.update()
            val (x, y, heading) = drive.poseEstimate
            telemetry.addData("x", x)
            telemetry.addData("y", y)
            telemetry.addData("heading", heading)
            telemetry.update()
        }
    }
}