package org.firstinspires.ftc.teamcode.commands.drive;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class ManualDriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;

    public ManualDriveCommand (DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
    }
}
