package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.EnhancedGamepad;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public abstract class RobotOpMode extends OpMode {
    public Drivetrain drive;
    public Intake intake;
    public Transfer transfer;
    public Launcher launcher;

    public Subsystem[] subsystems;
    public EnhancedGamepad gamer1 = new EnhancedGamepad(gamepad1);
    public EnhancedGamepad gamer2 = new EnhancedGamepad(gamepad2);

    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        launcher = new Launcher(hardwareMap);

        gamer1 = new EnhancedGamepad(gamepad1);
        gamer2 = new EnhancedGamepad(gamepad2);

        subsystems = new Subsystem[]{ drive, intake, transfer, launcher };
    }

    @Override
    public void loop() {
        for (Subsystem subsystem : subsystems) {
            subsystem.update();
        }
        gamer1.update();
        gamer2.update();
    }
}

