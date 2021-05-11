package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.Wobble;

public class RobotAuto extends LinearOpMode {
    public Drivetrain drive;
    public Intake intake;
    public Transfer transfer;
    public Launcher launcher;
    public Wobble wobble;
    public Vision vision;

    public Subsystem[] subsystems;

    private Runnable updateRunnable = new Runnable() {
        @Override
        public void run() {
            while (opModeIsActive()) {
                for (Subsystem subsystem : subsystems) {
                    if (subsystem.equals(drive)) {
                        if (drive.mode == Drivetrain.Mode.IDLE) {
                            subsystem.update();
                        }
                    } else {
                        subsystem.update();
                    }
                }
                telemetry.update();
            }
        }
    };

    protected Thread updateThread = new Thread(updateRunnable);

    @Override
    public void runOpMode() throws InterruptedException {

    }

    void initialize() {
        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        launcher = new Launcher(hardwareMap);
        wobble = new Wobble(hardwareMap);
        vision = new Vision(hardwareMap);

        subsystems = new Subsystem[]{ drive, intake, transfer, launcher, wobble, vision };
    }
}
